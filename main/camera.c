#include "camera.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"

static const char *TAG = "camera";

/* ---------------------------------------------------------------------------
 * Pinout DVP de la Waveshare ESP32-S3-CAM-OV5640 (SKU 33699).
 * Contrastado entre los ejemplos oficiales 04_dvp_camera_display y
 * 05_lvgl_brookesia, que coinciden pin a pin.
 * ------------------------------------------------------------------------ */
#define CAM_PIN_XCLK   GPIO_NUM_38
#define CAM_PIN_PCLK   GPIO_NUM_41
#define CAM_PIN_VSYNC  GPIO_NUM_17
#define CAM_PIN_HREF   GPIO_NUM_18
#define CAM_PIN_D0     GPIO_NUM_45
#define CAM_PIN_D1     GPIO_NUM_47
#define CAM_PIN_D2     GPIO_NUM_48
#define CAM_PIN_D3     GPIO_NUM_46
#define CAM_PIN_D4     GPIO_NUM_42
#define CAM_PIN_D5     GPIO_NUM_40
#define CAM_PIN_D6     GPIO_NUM_39
#define CAM_PIN_D7     GPIO_NUM_21

/* PWDN y RESET no estan cableados en esta variante de la placa. */
#define CAM_PIN_PWDN   GPIO_NUM_NC
#define CAM_PIN_RESET  GPIO_NUM_NC

/* SCCB: bus I2C compartido con el resto de perifericos, no uno dedicado. */
#define CAM_I2C_PORT   I2C_NUM_0
#define CAM_PIN_SCL    GPIO_NUM_7
#define CAM_PIN_SDA    GPIO_NUM_8

#if CONFIG_MIRILLA_FRAMESIZE_FHD
#define MIRILLA_FRAMESIZE      FRAMESIZE_FHD
#define MIRILLA_FRAMESIZE_STR  "1920x1080"
#elif CONFIG_MIRILLA_FRAMESIZE_HD
#define MIRILLA_FRAMESIZE      FRAMESIZE_HD
#define MIRILLA_FRAMESIZE_STR  "1280x720"
#else
#define MIRILLA_FRAMESIZE      FRAMESIZE_SVGA
#define MIRILLA_FRAMESIZE_STR  "800x600"
#endif

static i2c_master_bus_handle_t s_i2c_bus;
static int64_t s_exposure_guard_next_us;

/* ---------------------------------------------------------------------------
 * Politica de exposicion del OV5640.
 *
 * El sensor arranca con el modo nocturno activado (0x3a00 bit 2) y con los
 * limites del AEC que trae la tabla por defecto del driver esp32-camera,
 * calculados para otro modo de video. En 1080p JPEG con XCLK de 20MHz el modo
 * que corre de verdad es este:
 *
 *   SYSCLK 50MHz, HTS 2844, VTS 1488 -> linea 56.9us, frame 84.6ms (11.8 FPS)
 *
 * y los limites que quedan puestos, estos:
 *
 *   techo de exposicion (0x3a02/03 y 0x3a14/15) = 0x03d8 = 984 lineas = 56ms
 *   techo de ganancia   (0x3a18/19)             = 0x00f8 = 15.5x
 *   pasos de banda      (0x3a08..0x3a0b)        = 295 y 246 lineas, cuando los
 *                                                 reales aqui son 175 (50Hz)
 *                                                 y 146 (60Hz)
 *
 * O sea: el AEC no puede pasar de 56ms x 15.5x aunque el frame dure 84.6ms, y
 * encima cuantiza la exposicion con pasos de banda de otro modo. En la mirilla
 * eso pesa mas de lo que parece, porque la optica solo proyecta un circulo que
 * ocupa ~10% del frame y el resto es negro: la media que mide el AEC no puede
 * alcanzar su objetivo por mucho que exponga, asi que el control se queda
 * pegado al techo y la imagen es, literalmente, "lo que de el techo".
 *
 * Aqui se reprograman los limites para el modo que esta corriendo (formulas
 * del datasheet, las mismas que usa drivers/media/i2c/ov5640.c en Linux) y se
 * elige que hace el sensor cuando se queda sin luz:
 *
 *   MODO DIA    modo nocturno off, el frame nunca se alarga: dura siempre
 *               HTS x VTS. El ritmo es fijo pase lo que pase. Es el default.
 *   MODO NOCHE  modo nocturno on: el AEC alarga el frame con lineas de
 *               relleno para exponer mas tiempo. Cuesta FPS de verdad.
 *
 * Sobre el coste del modo noche, porque este comentario decia lo contrario y
 * costo 2.5 FPS: el techo de exposicion NO es un suelo de FPS.
 *
 *   1) A 1080p el driver DVP entrega un frame de cada DOS del sensor. Medido
 *      con el benchmark de captura pura en esta placa: frame de 84.63ms
 *      (11.81 FPS de sensor) y llegan 169ms clavados, min = max, 5.81 FPS.
 *      Alargar el frame del sensor 1ms cuesta 2ms de periodo.
 *   2) El bucle de subida es secuencial (esperar frame, subirlo), asi que
 *      encima el periodo es entrega + subida, no el mayor de los dos.
 *
 * De ahi salieron los 2.5 FPS: con el techo en 150ms el sensor se iba a ~200ms
 * de frame, el DVP lo doblaba a 400ms y el bucle quedaba clavado en 2.5. El
 * suelo real del modo noche no es 1000/MAX_EXPOSURE_MS, es la mitad.
 *
 * Por eso el techo se recorta al periodo del FPS objetivo dividido por esa
 * relacion, y por eso la duracion real del frame se lee del sensor y viaja en
 * la telemetria: con la placa montada en la puerta no hay USB, y sin ese dato
 * esto se manifiesta como un FPS bajo sin causa aparente.
 *
 * Registros implicados, todos del datasheet del OV5640:
 *   0x3034..0x3037, 0x3108  divisores del PLL, para calcular SYSCLK
 *   0x3808/0x380a           tamano de salida (para la ventana de medicion)
 *   0x380c/0x380e           HTS / VTS del modo actual
 *   0x3500..0x3502          exposicion en lineas (20 bits, 4 de fraccion)
 *   0x3503                  AEC/AGC en manual (distinto de 0 = no regula)
 *   0x350a/0x350b           ganancia real, formato 6.4 (16 unidades = 1x)
 *   0x350c/0x350d           lineas de relleno que anade el modo nocturno
 *   0x3a00                  bit 2 modo nocturno, bit 5 filtro de banda
 *   0x3a02/03 y 0x3a14/15   techo de exposicion (60Hz y 50Hz)
 *   0x3a08..0x3a0b          paso de banda de 50Hz y de 60Hz
 *   0x3a0d/0x3a0e           numero maximo de bandas (60Hz y 50Hz)
 *   0x3a0f,10,11,1b,1e,1f   nivel medio objetivo del AEC y sus margenes
 *   0x3a18/0x3a19           techo de ganancia
 *   0x5680..0x5687          ventana sobre la que el AEC promedia
 *   0x56a1                  media medida (solo lectura, para el diagnostico)
 * ------------------------------------------------------------------------ */
#define OV5640_REG_SC_PLL_CTRL0     0x3034
#define OV5640_REG_SC_PLL_CTRL1     0x3035
#define OV5640_REG_SC_PLL_CTRL2     0x3036
#define OV5640_REG_SC_PLL_CTRL3     0x3037
#define OV5640_REG_SYS_ROOT_DIV     0x3108
#define OV5640_REG_EXPOSURE_MID     0x3501
#define OV5640_REG_AEC_PK_MANUAL    0x3503
#define OV5640_REG_GAIN             0x350a
#define OV5640_REG_VTS_EXTRA        0x350c
#define OV5640_REG_X_OUTPUT_SIZE    0x3808
#define OV5640_REG_Y_OUTPUT_SIZE    0x380a
#define OV5640_REG_TIMING_HTS       0x380c
#define OV5640_REG_TIMING_VTS       0x380e
#define OV5640_REG_AEC_CTRL00       0x3a00
#define OV5640_NIGHT_MODE_BIT       0x04
#define OV5640_BAND_FILTER_BIT      0x20
#define OV5640_REG_AEC_MAX_EXP_60   0x3a02
#define OV5640_REG_AEC_B50_STEP     0x3a08
#define OV5640_REG_AEC_B60_STEP     0x3a0a
#define OV5640_REG_AEC_MAX_BAND_60  0x3a0d
#define OV5640_REG_AEC_MAX_BAND_50  0x3a0e
#define OV5640_REG_AEC_HIGH         0x3a0f
#define OV5640_REG_AEC_LOW          0x3a10
#define OV5640_REG_AEC_FAST_HIGH    0x3a11
#define OV5640_REG_AEC_MAX_EXP_50   0x3a14
#define OV5640_REG_AEC_GAIN_CEILING 0x3a18
#define OV5640_REG_AEC_HIGH2        0x3a1b
#define OV5640_REG_AEC_LOW2         0x3a1e
#define OV5640_REG_AEC_FAST_LOW     0x3a1f
#define OV5640_REG_AVG_X_START      0x5680
#define OV5640_REG_AVG_Y_START      0x5682
#define OV5640_REG_AVG_X_WINDOW     0x5684
#define OV5640_REG_AVG_Y_WINDOW     0x5686
#define OV5640_REG_AVG_READOUT      0x56a1

/* El techo de exposicion y el VTS son campos de 12 bits en el sensor. */
#define OV5640_MAX_EXPOSURE_LINES   0x0fff
/*
 * Frames del sensor por cada frame que entrega el driver DVP. A 1080p es 2,
 * medido: el sensor va a 84.63ms y esp_camera_fb_get() devuelve uno cada
 * 169ms, sin dispersion. Es lo que convierte la duracion del frame en el techo
 * de FPS del aparato entero.
 */
#define MIRILLA_SENSOR_FRAMES_PER_CAPTURE 2
/* Ganancia real en formato 6.4: 16 unidades = 1x, tope del registro 0x3ff. */
#define OV5640_GAIN_UNITS_PER_X     16
#define OV5640_MAX_GAIN_UNITS       0x03ff

#if CONFIG_MIRILLA_EXPOSURE_NIGHT
#define MIRILLA_NIGHT_MODE     1
#define MIRILLA_EXPOSURE_STR   "noche"
#else
#define MIRILLA_NIGHT_MODE     0
#define MIRILLA_EXPOSURE_STR   "dia"
#endif

#if CONFIG_MIRILLA_BANDING_FILTER
#define MIRILLA_BAND_FILTER    1
#else
#define MIRILLA_BAND_FILTER    0
#endif

/* Parametros del modo que esta corriendo, leidos del sensor una sola vez. */
typedef struct {
    uint32_t sysclk_hz;
    uint16_t hts;
    uint16_t vts;
    uint16_t out_w;
    uint16_t out_h;
    uint32_t line_ns;
    uint16_t max_exposure_lines;
} exposure_mode_t;

static exposure_mode_t s_mode;
static mirilla_camera_aec_t s_aec;

static int reg_read8(sensor_t *sensor, uint16_t reg)
{
    return sensor->get_reg(sensor, reg, 0xff);
}

static int reg_read16(sensor_t *sensor, uint16_t reg)
{
    return sensor->get_reg(sensor, reg, 0xffff);
}

/*
 * Escribe solo si hace falta y relee para confirmar: una escritura SCCB
 * perdida no debe pasar inadvertida. Devuelve false si no se pudo dejar el
 * registro como se pedia, y marca `fixed` si hubo que tocarlo (al arrancar es
 * lo normal; mas tarde significa que algo lo habia cambiado).
 */
static bool reg_ensure(sensor_t *sensor, uint16_t reg, uint16_t mask, uint16_t value, bool *fixed)
{
    const int current = sensor->get_reg(sensor, reg, mask);
    if (current < 0) {
        return false;
    }
    if ((current & mask) == (value & mask)) {
        return true;
    }
    if (fixed != NULL) {
        *fixed = true;
    }
    if (sensor->set_reg(sensor, reg, mask, value) < 0) {
        return false;
    }
    const int after = sensor->get_reg(sensor, reg, mask);
    return after >= 0 && (after & mask) == (value & mask);
}

/*
 * SYSCLK a partir de los divisores del PLL, tal cual lo documenta OmniVision.
 * De el salen el reloj de linea, el techo de exposicion en lineas y los pasos
 * del filtro de banda; sin calcularlo hay que adivinarlos, que es justo lo que
 * hace la tabla por defecto del driver.
 */
static uint32_t exposure_sysclk_hz(sensor_t *sensor)
{
    static const uint32_t sclk_rdiv_map[] = { 1, 2, 4, 8 };

    const int ctrl0 = reg_read8(sensor, OV5640_REG_SC_PLL_CTRL0);
    const int ctrl1 = reg_read8(sensor, OV5640_REG_SC_PLL_CTRL1);
    const int ctrl2 = reg_read8(sensor, OV5640_REG_SC_PLL_CTRL2);
    const int ctrl3 = reg_read8(sensor, OV5640_REG_SC_PLL_CTRL3);
    const int root = reg_read8(sensor, OV5640_REG_SYS_ROOT_DIV);
    if (ctrl0 < 0 || ctrl1 < 0 || ctrl2 < 0 || ctrl3 < 0 || root < 0) {
        return 0;
    }

    /* 0x3034[3:0]: 8 y 10 son bits por pixel y dividen; el resto no. */
    const uint32_t bits = (uint32_t) (ctrl0 & 0x0f);
    const uint32_t bit_div2x = (bits == 8 || bits == 10) ? bits / 2 : 1;
    const uint32_t sysdiv = (ctrl1 >> 4) ? (uint32_t) (ctrl1 >> 4) : 16;
    const uint32_t multiplier = (uint32_t) ctrl2;
    const uint32_t prediv = (uint32_t) (ctrl3 & 0x0f);
    const uint32_t pll_rdiv = (uint32_t) ((ctrl3 >> 4) & 0x01) + 1;
    const uint32_t sclk_rdiv = sclk_rdiv_map[root & 0x03];

    if (prediv == 0 || multiplier == 0) {
        return 0;
    }

    /* En unidades de 10kHz para que el VCO no desborde 32 bits. */
    const uint32_t xvclk = (uint32_t) CONFIG_MIRILLA_XCLK_FREQ_HZ / 10000;
    const uint32_t vco = xvclk * multiplier / prediv;
    return vco / sysdiv / pll_rdiv * 2 / bit_div2x / sclk_rdiv * 10000;
}

/* Geometria y reloj del modo actual, leidos del propio sensor. */
static bool exposure_mode_read(sensor_t *sensor, exposure_mode_t *mode)
{
    const uint32_t sysclk_hz = exposure_sysclk_hz(sensor);
    const int hts = reg_read16(sensor, OV5640_REG_TIMING_HTS);
    const int vts = reg_read16(sensor, OV5640_REG_TIMING_VTS);
    const int out_w = reg_read16(sensor, OV5640_REG_X_OUTPUT_SIZE);
    const int out_h = reg_read16(sensor, OV5640_REG_Y_OUTPUT_SIZE);
    if (sysclk_hz == 0 || hts <= 0 || vts <= 4 || out_w <= 0 || out_h <= 0) {
        return false;
    }

    mode->sysclk_hz = sysclk_hz;
    mode->hts = (uint16_t) (hts & 0x0fff);
    mode->vts = (uint16_t) (vts & 0x0fff);
    mode->out_w = (uint16_t) (out_w & 0x0fff);
    mode->out_h = (uint16_t) (out_h & 0x0fff);
    mode->line_ns = (uint32_t) ((uint64_t) mode->hts * 1000000000ULL / sysclk_hz);
    if (mode->line_ns == 0 || mode->hts == 0 || mode->vts <= 4) {
        return false;
    }

    /*
     * Techo de exposicion. En modo dia no tiene sentido pedir mas de un frame:
     * el sensor no puede exponer mas de VTS-4 lineas sin alargarlo, y
     * alargarlo es precisamente lo que hace el modo nocturno.
     *
     * En modo noche el recorte es el periodo del FPS objetivo repartido entre
     * los frames de sensor que cuesta cada captura. No garantiza ese ritmo (la
     * subida va detras, en el mismo bucle), pero impide lo que no tiene
     * defensa posible: pedir una exposicion que por si sola ya se pasa del
     * periodo objetivo.
     */
    uint32_t lines = (uint32_t) CONFIG_MIRILLA_MAX_EXPOSURE_MS * 1000000UL / mode->line_ns;
    if (!MIRILLA_NIGHT_MODE && lines > (uint32_t) mode->vts - 4) {
        lines = (uint32_t) mode->vts - 4;
    }
    if (MIRILLA_NIGHT_MODE) {
        const uint32_t budget_us = 1000000UL / (uint32_t) CONFIG_MIRILLA_TARGET_FPS
                                   / MIRILLA_SENSOR_FRAMES_PER_CAPTURE;
        const uint32_t budget_lines = budget_us * 1000UL / mode->line_ns;
        if (lines > budget_lines) {
            ESP_LOGW(TAG, "techo de exposicion recortado de %ums a %lums: a %d FPS cada "
                          "captura son %d frames de sensor, y la subida va despues",
                     (unsigned) CONFIG_MIRILLA_MAX_EXPOSURE_MS,
                     (unsigned long) (budget_us / 1000), CONFIG_MIRILLA_TARGET_FPS,
                     MIRILLA_SENSOR_FRAMES_PER_CAPTURE);
            lines = budget_lines;
        }
    }
    if (lines > OV5640_MAX_EXPOSURE_LINES) {
        lines = OV5640_MAX_EXPOSURE_LINES;
    }
    if (lines < 2) {
        lines = 2;
    }
    mode->max_exposure_lines = (uint16_t) lines;
    return true;
}

/* Duracion en microsegundos de un numero de lineas del modo actual. */
static uint32_t exposure_lines_to_us(uint32_t lines)
{
    return (uint32_t) ((uint64_t) lines * s_mode.line_ns / 1000);
}

/*
 * Deja el bloque AEC/AGC como pide la configuracion. Idempotente a proposito:
 * se llama al arrancar y luego cada MIRILLA_EXPOSURE_GUARD_S segundos, y en el
 * caso normal son solo lecturas.
 */
static bool exposure_policy_apply(sensor_t *sensor, bool *fixed)
{
    if (fixed != NULL) {
        *fixed = false;
    }
    if (sensor == NULL || sensor->get_reg == NULL || sensor->set_reg == NULL) {
        return false;
    }
    if (s_mode.line_ns == 0 && !exposure_mode_read(sensor, &s_mode)) {
        return false;
    }

    bool ok = true;

    /* Techo de exposicion, en las dos variantes (60Hz y 50Hz) que mira el AEC. */
    ok &= reg_ensure(sensor, OV5640_REG_AEC_MAX_EXP_60, 0x0fff, s_mode.max_exposure_lines, fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AEC_MAX_EXP_50, 0x0fff, s_mode.max_exposure_lines, fixed);

    /*
     * Filtro de banda: cuantiza la exposicion en multiplos de medio ciclo de
     * la red para que la luz artificial no deje bandas horizontales. El paso
     * depende del reloj de linea, asi que hay que recalcularlo para este modo,
     * y el numero maximo de bandas es lo que limita la exposicion mientras el
     * filtro esta activo.
     */
#if MIRILLA_BAND_FILTER
    const uint32_t band_step_50 = s_mode.sysclk_hz / ((uint32_t) s_mode.hts * 100);
    const uint32_t band_step_60 = s_mode.sysclk_hz / ((uint32_t) s_mode.hts * 120);
    if (band_step_50 == 0 || band_step_60 == 0) {
        ok = false;
    } else {
        uint32_t max_band_50 = s_mode.max_exposure_lines / band_step_50;
        uint32_t max_band_60 = s_mode.max_exposure_lines / band_step_60;
        if (max_band_50 < 1) max_band_50 = 1;
        if (max_band_60 < 1) max_band_60 = 1;
        if (max_band_50 > 0xff) max_band_50 = 0xff;
        if (max_band_60 > 0xff) max_band_60 = 0xff;

        ok &= reg_ensure(sensor, OV5640_REG_AEC_B50_STEP, 0xffff, (uint16_t) band_step_50, fixed);
        ok &= reg_ensure(sensor, OV5640_REG_AEC_B60_STEP, 0xffff, (uint16_t) band_step_60, fixed);
        ok &= reg_ensure(sensor, OV5640_REG_AEC_MAX_BAND_50, 0xff, (uint16_t) max_band_50, fixed);
        ok &= reg_ensure(sensor, OV5640_REG_AEC_MAX_BAND_60, 0xff, (uint16_t) max_band_60, fixed);
    }
#endif
    ok &= reg_ensure(sensor, OV5640_REG_AEC_CTRL00, OV5640_BAND_FILTER_BIT,
                     MIRILLA_BAND_FILTER ? OV5640_BAND_FILTER_BIT : 0, fixed);

    /* Techo de ganancia analogica, en formato 6.4. */
    uint32_t gain_units = (uint32_t) CONFIG_MIRILLA_GAIN_CEILING_X * OV5640_GAIN_UNITS_PER_X;
    if (gain_units > OV5640_MAX_GAIN_UNITS) {
        gain_units = OV5640_MAX_GAIN_UNITS;
    }
    ok &= reg_ensure(sensor, OV5640_REG_AEC_GAIN_CEILING, 0x03ff, (uint16_t) gain_units, fixed);

    /*
     * Nivel medio objetivo del AEC y su histeresis, con las proporciones que
     * documenta OmniVision: +-8% de zona estable, y el doble y la mitad para
     * decidir cuando corregir de golpe en vez de poco a poco.
     */
    uint32_t high = (uint32_t) CONFIG_MIRILLA_AEC_TARGET * 27 / 25;
    uint32_t low = (uint32_t) CONFIG_MIRILLA_AEC_TARGET * 23 / 25;
    if (high > 0xff) high = 0xff;
    if (low < 1) low = 1;
    uint32_t fast_high = high * 2;
    if (fast_high > 0xff) fast_high = 0xff;
    const uint32_t fast_low = low / 2;

    ok &= reg_ensure(sensor, OV5640_REG_AEC_HIGH, 0xff, (uint16_t) high, fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AEC_LOW, 0xff, (uint16_t) low, fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AEC_HIGH2, 0xff, (uint16_t) high, fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AEC_LOW2, 0xff, (uint16_t) low, fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AEC_FAST_HIGH, 0xff, (uint16_t) fast_high, fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AEC_FAST_LOW, 0xff, (uint16_t) fast_low, fixed);

    /*
     * Ventana de medicion. El AEC promedia esta region y la compara con el
     * objetivo; por defecto abarca el frame entero, y en la mirilla eso es
     * medir sobre todo el negro que rodea al circulo de la optica.
     *
     * Aqui esta el detalle que se llevo por delante la exposicion del modo
     * dia, asi que conviene dejarlo escrito con numeros. El circulo util no es
     * "aproximadamente el centro del frame": el servidor lo tiene medido para
     * su mascara de deteccion (MOTION_ROI_CX/CY/R en storage.py) y a 1080p son
     *
     *   centro (960, 508) y radio 227 px  ->  un disco de 161.879 px
     *
     * o sea que esta 32 px por encima del centro del frame. Con la ventana en
     * el 55% del alto (594x594 = 352.836 px) solo el 46% de lo que promedia el
     * AEC es imagen; el 54% restante es el tunel negro de la mirilla, que no
     * se aclara por mucho que se exponga. Para que la media de la VENTANA
     * llegue al objetivo, la media del DISCO tiene que subir a
     *
     *   objetivo / 0.459 = 64 -> 139 de 255
     *
     * y eso es exactamente lo que se veia: de dia el AEC quema el circulo
     * entero persiguiendo un 139 que solo alcanza saturando, y al anochecer
     * pide un 139 que no existe, se clava en el techo de exposicion y de
     * ganancia y deja de regular. El mismo error explica las dos cosas.
     *
     * La ventana correcta es el cuadrado inscrito en el disco. Sin
     * desplazamiento vertical el lado maximo que cabe entero es 280 px (26%
     * del alto): la esquina peor cae a 221.8 px del centro del disco, dentro
     * de los 227 de radio. Centrandola de verdad en y=508 caben 227*raiz(2) =
     * 321 px (30%), que es lo que da MIRILLA_METER_OFFSET_Y_PCT.
     *
     * Ese offset va aparte y por defecto a 0 a proposito: con vflip+hmirror
     * activos no esta escrito en ningun sitio si las coordenadas del bloque
     * AVG son anteriores o posteriores al giro, y con el lado por defecto la
     * ventana cae dentro del disco con cualquiera de los dos signos. Para
     * afinarlo hay que medirlo: subir el offset y mirar si `avg` sube o baja.
     */
#if CONFIG_MIRILLA_METER_CENTER_PCT > 0
    uint32_t side = (uint32_t) s_mode.out_h * CONFIG_MIRILLA_METER_CENTER_PCT / 100;
    if (side > s_mode.out_w) {
        side = s_mode.out_w;
    }
    if (side > s_mode.out_h) {
        side = s_mode.out_h;
    }
    if (side < 16) {
        side = 16;
    }

    /* Centrada, y luego desplazada en vertical sin salirse del frame. */
    int32_t y_start = ((int32_t) s_mode.out_h - (int32_t) side) / 2
                      + (int32_t) s_mode.out_h * CONFIG_MIRILLA_METER_OFFSET_Y_PCT / 100;
    if (y_start < 0) {
        y_start = 0;
    }
    if (y_start > (int32_t) (s_mode.out_h - side)) {
        y_start = (int32_t) (s_mode.out_h - side);
    }

    ok &= reg_ensure(sensor, OV5640_REG_AVG_X_START, 0xffff,
                     (uint16_t) ((s_mode.out_w - side) / 2), fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AVG_Y_START, 0xffff, (uint16_t) y_start, fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AVG_X_WINDOW, 0xffff, (uint16_t) side, fixed);
    ok &= reg_ensure(sensor, OV5640_REG_AVG_Y_WINDOW, 0xffff, (uint16_t) side, fixed);
#endif

    /* Y al final el modo nocturno, que es quien decide si el frame se alarga. */
    ok &= reg_ensure(sensor, OV5640_REG_AEC_CTRL00, OV5640_NIGHT_MODE_BIT,
                     MIRILLA_NIGHT_MODE ? OV5640_NIGHT_MODE_BIT : 0, fixed);
    if (!MIRILLA_NIGHT_MODE) {
        /* Sin modo nocturno nadie deberia estar anadiendo lineas de relleno. */
        ok &= reg_ensure(sensor, OV5640_REG_VTS_EXTRA, 0xffff, 0, fixed);
    }

    if (ok) {
        /* Que el status del driver no contradiga a los registros. */
        sensor->status.aec2 = MIRILLA_NIGHT_MODE;
    }
    return ok;
}

/*
 * Que ha elegido el AEC ahora mismo. Es el diagnostico que dice si la escena
 * sale oscura porque no hay luz o porque el control esta topado: si exposicion
 * y ganancia estan en su techo y la media medida sigue por debajo del
 * objetivo, es lo segundo y hay margen subiendo los techos.
 *
 * `frame_ms` es ademas el unico sitio donde se ve la duracion real del frame,
 * que es lo que fija el techo de FPS de todo el aparato. Por eso esto se lee
 * siempre, se registre o no en el log: viaja en la telemetria, y con la placa
 * montada en la puerta la telemetria es la unica ventana que hay.
 */
static void exposure_sample_state(sensor_t *sensor)
{
    /*
     * 0x3501/0x3502 son los 16 bits bajos del campo de 20: lineas en los 12
     * altos y fraccion de linea en los 4 bajos. Con el techo limitado a 12
     * bits de lineas, 0x3500 siempre es cero y no hace falta leerlo.
     */
    const int exposure = reg_read16(sensor, OV5640_REG_EXPOSURE_MID);
    const int gain = reg_read16(sensor, OV5640_REG_GAIN);
    const int vts_extra = reg_read16(sensor, OV5640_REG_VTS_EXTRA);
    const int avg = reg_read8(sensor, OV5640_REG_AVG_READOUT);
    const int manual = reg_read8(sensor, OV5640_REG_AEC_PK_MANUAL);
    const int ctrl00 = reg_read8(sensor, OV5640_REG_AEC_CTRL00);
    if (exposure < 0 || gain < 0 || vts_extra < 0 || avg < 0 || manual < 0 || ctrl00 < 0) {
        s_aec.valid = false;
        ESP_LOGW(TAG, "no se pudo leer el estado del AEC por SCCB");
        return;
    }

    const uint32_t exposure_us = exposure_lines_to_us((uint32_t) exposure >> 4);
    const uint32_t ceiling_us = exposure_lines_to_us(s_mode.max_exposure_lines);
    const uint32_t frame_us = exposure_lines_to_us((uint32_t) s_mode.vts + (uint32_t) vts_extra);
    const uint32_t gain_x100 = (uint32_t) (gain & 0x03ff) * 100 / OV5640_GAIN_UNITS_PER_X;

    s_aec.exposure_ms = (uint16_t) ((exposure_us + 500) / 1000);
    s_aec.ceiling_ms = (uint16_t) ((ceiling_us + 500) / 1000);
    s_aec.frame_ms = (uint16_t) ((frame_us + 500) / 1000);
    s_aec.gain_x100 = (uint16_t) gain_x100;
    s_aec.avg = (uint8_t) avg;
    s_aec.manual = (uint8_t) manual;
    s_aec.ctrl00 = (uint8_t) ctrl00;
    s_aec.valid = true;

    /*
     * Nadie de este proyecto escribe 0x3503, asi que si algun dia sale
     * distinto de cero es que lo ha hecho el driver (set_exposure_ctrl o
     * set_gain_ctrl) y el AEC lleva congelado desde entonces: la imagen se
     * queda como estaba pase lo que pase con la luz. Merece un aviso propio
     * porque es la unica averia que no se distingue de "no hay luz" mirando
     * exposicion y ganancia.
     */
    if (manual != 0) {
        ESP_LOGW(TAG, "0x3503 = 0x%02x: el AEC/AGC esta en MANUAL y no va a regular", manual);
    }

#if CONFIG_MIRILLA_EXPOSURE_LOG
    ESP_LOGI(TAG,
             "AEC: exposicion %lu.%02lu ms de %lu.%02lu ms, ganancia %lu.%02lux de %dx, "
             "media medida %d/255 (objetivo %d), frame %lu.%02lu ms (%lu.%02lu FPS), "
             "0x3503=0x%02x 0x3a00=0x%02x",
             (unsigned long) exposure_us / 1000, (unsigned long) (exposure_us % 1000) / 10,
             (unsigned long) ceiling_us / 1000, (unsigned long) (ceiling_us % 1000) / 10,
             (unsigned long) gain_x100 / 100, (unsigned long) gain_x100 % 100,
             CONFIG_MIRILLA_GAIN_CEILING_X, avg, CONFIG_MIRILLA_AEC_TARGET,
             (unsigned long) frame_us / 1000, (unsigned long) (frame_us % 1000) / 10,
             (unsigned long) (frame_us ? 1000000UL / frame_us : 0),
             (unsigned long) (frame_us ? (100000000UL / frame_us) % 100 : 0),
             manual, ctrl00);
#endif
}

void mirilla_camera_aec(mirilla_camera_aec_t *out)
{
    if (out != NULL) {
        *out = s_aec;
    }
}

void mirilla_camera_keep_exposure_policy(void)
{
#if CONFIG_MIRILLA_EXPOSURE_GUARD_S > 0
    const int64_t now = esp_timer_get_time();
    if (now < s_exposure_guard_next_us) {
        return;
    }
    s_exposure_guard_next_us = now + (int64_t) CONFIG_MIRILLA_EXPOSURE_GUARD_S * 1000000LL;

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        return;
    }

    bool fixed = false;
    if (!exposure_policy_apply(sensor, &fixed)) {
        ESP_LOGW(TAG, "no se pudo reafirmar la politica de exposicion por SCCB");
    } else if (fixed) {
        ESP_LOGW(TAG, "la politica de exposicion habia cambiado, reescrita (modo %s)",
                 MIRILLA_EXPOSURE_STR);
    }
    exposure_sample_state(sensor);
#endif
}

const char *mirilla_camera_resolution_str(void)
{
    return MIRILLA_FRAMESIZE_STR;
}

/*
 * El driver esp32-camera compilado contra IDF >= 5.4 usa sccb-ng.c, que
 * recupera el bus con i2c_master_get_bus_handle(port). Por eso hay que crear
 * el bus con la API nueva (i2c_new_master_bus) y no con la legacy.
 */
static esp_err_t sccb_bus_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = CAM_I2C_PORT,
        .sda_io_num = CAM_PIN_SDA,
        .scl_io_num = CAM_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus fallo: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "bus I2C%d listo (SDA=%d SCL=%d)", CAM_I2C_PORT, CAM_PIN_SDA, CAM_PIN_SCL);
    return ESP_OK;
}

esp_err_t mirilla_camera_init(void)
{
    esp_err_t err = sccb_bus_init();
    if (err != ESP_OK) {
        return err;
    }

    const camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        /* -1 en sda le dice al driver que use sccb_i2c_port en vez de pines. */
        .pin_sccb_sda = GPIO_NUM_NC,
        .pin_sccb_scl = GPIO_NUM_NC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = CONFIG_MIRILLA_XCLK_FREQ_HZ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = MIRILLA_FRAMESIZE,
        .jpeg_quality = CONFIG_MIRILLA_JPEG_QUALITY,
        .fb_count = CONFIG_MIRILLA_FB_COUNT,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
        .sccb_i2c_port = CAM_I2C_PORT,
    };

    err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init fallo: %s (0x%x)", esp_err_to_name(err), err);
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "no se pudo obtener el handle del sensor");
        return ESP_FAIL;
    }

    /* Reintentos: los primeros accesos SCCB tras configurar el sensor a veces
       fallan, y de esto no queremos quedarnos sin saber si cuajo. */
    bool applied = false;
    for (int attempt = 1; attempt <= 3 && !applied; attempt++) {
        applied = exposure_policy_apply(sensor, NULL);
        if (!applied) {
            ESP_LOGW(TAG, "intento %d de aplicar la politica de exposicion sin confirmar", attempt);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    if (applied) {
        ESP_LOGI(TAG, "modo del sensor: SYSCLK %lu MHz, HTS %u, VTS %u, linea %lu.%02lu us, "
                      "frame base %lu.%02lu ms",
                 (unsigned long) (s_mode.sysclk_hz / 1000000), s_mode.hts, s_mode.vts,
                 (unsigned long) s_mode.line_ns / 1000, (unsigned long) (s_mode.line_ns % 1000) / 10,
                 (unsigned long) exposure_lines_to_us(s_mode.vts) / 1000,
                 (unsigned long) (exposure_lines_to_us(s_mode.vts) % 1000) / 10);
        ESP_LOGI(TAG, "exposicion en modo %s: techo %lu.%02lu ms (%u lineas), ganancia hasta %dx, "
                      "objetivo AEC %d/255, filtro de banda %s, revision cada %ds",
                 MIRILLA_EXPOSURE_STR,
                 (unsigned long) exposure_lines_to_us(s_mode.max_exposure_lines) / 1000,
                 (unsigned long) (exposure_lines_to_us(s_mode.max_exposure_lines) % 1000) / 10,
                 s_mode.max_exposure_lines, CONFIG_MIRILLA_GAIN_CEILING_X,
                 CONFIG_MIRILLA_AEC_TARGET,
                 MIRILLA_BAND_FILTER ? "on" : "off",
                 CONFIG_MIRILLA_EXPOSURE_GUARD_S);

        /*
         * La ventana se relee del sensor en vez de recalcularla: es el unico
         * sitio donde se ve lo que quedo escrito de verdad, y es el ajuste del
         * que depende que el AEC mida la escena o el tunel negro.
         */
        const int win_x = reg_read16(sensor, OV5640_REG_AVG_X_START);
        const int win_y = reg_read16(sensor, OV5640_REG_AVG_Y_START);
        const int win_w = reg_read16(sensor, OV5640_REG_AVG_X_WINDOW);
        const int win_h = reg_read16(sensor, OV5640_REG_AVG_Y_WINDOW);
        if (win_x < 0 || win_y < 0 || win_w <= 0 || win_h <= 0) {
            ESP_LOGW(TAG, "no se pudo releer la ventana de medicion del AEC");
        } else {
            ESP_LOGI(TAG, "ventana de medicion: %dx%d en (%d,%d), centro (%d,%d); "
                          "el circulo de la optica esta en (%u,%u) con radio %u",
                     win_w, win_h, win_x, win_y,
                     win_x + win_w / 2, win_y + win_h / 2,
                     (unsigned) (s_mode.out_w / 2), (unsigned) (s_mode.out_h * 47 / 100),
                     (unsigned) (s_mode.out_h * 21 / 100));
        }
    } else {
        ESP_LOGE(TAG, "politica de exposicion NO confirmada: el sensor se queda con los "
                      "limites por defecto del driver y la escena saldra oscura");
    }
    s_exposure_guard_next_us = esp_timer_get_time() +
                               (int64_t) CONFIG_MIRILLA_EXPOSURE_GUARD_S * 1000000LL;

#if CONFIG_MIRILLA_ROTATE_180
    /*
     * La placa va montada boca abajo en la mirilla, asi que la imagen sale
     * invertida. Un giro de 180 grados es vflip + hmirror, y el OV5640 los
     * aplica en su propio ISP antes de comprimir: son escrituras I2C una sola
     * vez al arrancar, sin coste por frame ni perdida de calidad. La
     * alternativa (girarlo en el servidor) obligaria a descodificar y
     * recomprimir cada JPEG de 1080p.
     */
    if (sensor->set_vflip == NULL || sensor->set_hmirror == NULL) {
        ESP_LOGW(TAG, "el sensor no expone vflip/hmirror, imagen sin girar");
    } else if (sensor->set_vflip(sensor, 1) != 0 || sensor->set_hmirror(sensor, 1) != 0) {
        ESP_LOGW(TAG, "no se pudo girar la imagen 180 grados en el sensor");
    } else {
        ESP_LOGI(TAG, "imagen girada 180 grados en el sensor (vflip + hmirror)");
    }
#endif

    camera_sensor_info_t *info = esp_camera_sensor_get_info(&sensor->id);
    ESP_LOGI(TAG, "sensor %s (PID 0x%04x) inicializado a %s, calidad JPEG %d, %d framebuffers",
             info ? info->name : "desconocido", sensor->id.PID,
             MIRILLA_FRAMESIZE_STR, CONFIG_MIRILLA_JPEG_QUALITY, CONFIG_MIRILLA_FB_COUNT);
    ESP_LOGI(TAG, "PSRAM libre tras init: %u bytes",
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    return ESP_OK;
}

void mirilla_camera_benchmark(int seconds)
{
    if (seconds <= 0) {
        return;
    }

    ESP_LOGI(TAG, "benchmark de captura pura durante %d s...", seconds);

    const int64_t start = esp_timer_get_time();
    const int64_t deadline = start + (int64_t) seconds * 1000000LL;
    int64_t prev = start;
    int64_t min_gap = INT64_MAX, max_gap = 0;
    uint32_t frames = 0;
    uint64_t bytes = 0;

    while (esp_timer_get_time() < deadline) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "benchmark: fb_get devolvio NULL");
            continue;
        }
        const int64_t now = esp_timer_get_time();
        const int64_t gap = now - prev;
        if (frames > 0) {
            if (gap < min_gap) min_gap = gap;
            if (gap > max_gap) max_gap = gap;
        }
        prev = now;
        frames++;
        bytes += fb->len;
        esp_camera_fb_return(fb);
    }

    const double elapsed = (double) (esp_timer_get_time() - start) / 1000000.0;
    ESP_LOGI(TAG,
             "benchmark: %.2f FPS de captura pura (%lu frames en %.1fs), "
             "JPEG medio %.1f KB, intervalo min %lld ms / max %lld ms",
             frames / elapsed, (unsigned long) frames, elapsed,
             frames ? (double) bytes / frames / 1024.0 : 0.0,
             min_gap == INT64_MAX ? 0 : min_gap / 1000, max_gap / 1000);

    /* Tras el benchmark el AEC ya ha convergido: es el mejor momento para ver
       con que exposicion y ganancia se ha quedado. */
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != NULL && s_mode.line_ns != 0) {
        exposure_sample_state(sensor);
    }
}
