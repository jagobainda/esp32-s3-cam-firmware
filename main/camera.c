#include "camera.h"

#include <stdbool.h>

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
static int64_t s_night_guard_next_us;

/* ---------------------------------------------------------------------------
 * Modo nocturno del OV5640.
 *
 * AEC_CTRL00 (0x3a00) bit 2 lo habilita: en escenas oscuras el AEC alarga el
 * frame insertando lineas de relleno (VTS_EXTRA, 0x350c/0x350d) para exponer
 * mas tiempo. Sale una imagen borrosa y el ritmo se hunde a ~4 FPS. Medido en
 * hardware: con el activado no pasaba de 3.9 FPS ni subiendo XCLK ni bajando
 * la calidad JPEG.
 *
 * No basta con escribir el bit una vez al arrancar: una escritura SCCB perdida
 * pasaria inadvertida. Aqui se escribe, se relee para confirmar, y luego se
 * vigila cada MIRILLA_NIGHT_MODE_GUARD_S segundos. Las lineas extra son el
 * efecto observable del modo nocturno, asi que se comprueban tambien: si no
 * son cero el frame ya viene alargado, sea quien sea el que lo hizo.
 * ------------------------------------------------------------------------ */
#define OV5640_REG_AEC_CTRL00   0x3a00
#define OV5640_NIGHT_MODE_BIT   0x04
#define OV5640_REG_VTS_EXTRA_H  0x350c
#define OV5640_REG_VTS_EXTRA_L  0x350d

/*
 * Deja el modo nocturno desactivado. Devuelve true si al salir se ha podido
 * confirmar por lectura que lo esta; `fixed`, si no es NULL, indica si hubo que
 * corregir algo (al arrancar es normal, mas tarde significa que habia vuelto).
 */
static bool night_mode_force_off(sensor_t *sensor, bool *fixed)
{
    if (fixed != NULL) {
        *fixed = false;
    }
    if (sensor == NULL || sensor->get_reg == NULL || sensor->set_reg == NULL) {
        return false;
    }

    bool ok = true;

    int aec = sensor->get_reg(sensor, OV5640_REG_AEC_CTRL00, 0xff);
    if (aec < 0) {
        ok = false;
    } else if (aec & OV5640_NIGHT_MODE_BIT) {
        if (fixed != NULL) {
            *fixed = true;
        }
        if (sensor->set_reg(sensor, OV5640_REG_AEC_CTRL00, OV5640_NIGHT_MODE_BIT, 0) < 0) {
            ok = false;
        } else {
            aec = sensor->get_reg(sensor, OV5640_REG_AEC_CTRL00, 0xff);
            if (aec < 0 || (aec & OV5640_NIGHT_MODE_BIT) != 0) {
                ok = false;
            }
        }
    }

    const int extra_h = sensor->get_reg(sensor, OV5640_REG_VTS_EXTRA_H, 0xff);
    const int extra_l = sensor->get_reg(sensor, OV5640_REG_VTS_EXTRA_L, 0xff);
    if (extra_h < 0 || extra_l < 0) {
        ok = false;
    } else if (extra_h != 0 || extra_l != 0) {
        if (fixed != NULL) {
            *fixed = true;
        }
        if (sensor->set_reg(sensor, OV5640_REG_VTS_EXTRA_H, 0xff, 0) < 0) {
            ok = false;
        }
        if (sensor->set_reg(sensor, OV5640_REG_VTS_EXTRA_L, 0xff, 0) < 0) {
            ok = false;
        }
    }

    /* Que el status del driver no contradiga al registro. */
    if (ok) {
        sensor->status.aec2 = 0;
    }
    return ok;
}

void mirilla_camera_keep_night_mode_off(void)
{
#if CONFIG_MIRILLA_NIGHT_MODE_GUARD_S > 0
    const int64_t now = esp_timer_get_time();
    if (now < s_night_guard_next_us) {
        return;
    }
    s_night_guard_next_us = now + (int64_t) CONFIG_MIRILLA_NIGHT_MODE_GUARD_S * 1000000LL;

    sensor_t *sensor = esp_camera_sensor_get();
    bool fixed = false;
    if (!night_mode_force_off(sensor, &fixed)) {
        ESP_LOGW(TAG, "no se pudo comprobar el estado del modo nocturno por SCCB");
    } else if (fixed) {
        ESP_LOGW(TAG, "el modo nocturno habia reaparecido, forzado a off otra vez");
    } else {
        ESP_LOGD(TAG, "vigilancia: modo nocturno sigue off (AEC_CTRL00=0x%02x)",
                 sensor->get_reg(sensor, OV5640_REG_AEC_CTRL00, 0xff));
    }
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
       fallan, y de este no queremos quedarnos sin saber si cuajo. */
    bool night_off = false;
    for (int attempt = 1; attempt <= 3 && !night_off; attempt++) {
        night_off = night_mode_force_off(sensor, NULL);
        if (!night_off) {
            ESP_LOGW(TAG, "intento %d de desactivar el modo nocturno sin confirmar", attempt);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    if (night_off) {
        ESP_LOGI(TAG, "modo nocturno forzado a off y confirmado por lectura "
                      "(AEC_CTRL00=0x%02x, VTS extra=%d), revision cada %ds",
                 sensor->get_reg(sensor, OV5640_REG_AEC_CTRL00, 0xff),
                 (sensor->get_reg(sensor, OV5640_REG_VTS_EXTRA_H, 0xff) << 8) |
                     sensor->get_reg(sensor, OV5640_REG_VTS_EXTRA_L, 0xff),
                 CONFIG_MIRILLA_NIGHT_MODE_GUARD_S);
    } else {
        ESP_LOGE(TAG, "modo nocturno NO confirmado como off: en escenas oscuras "
                      "el sensor puede alargar el frame y hundir el ritmo");
    }
    s_night_guard_next_us = esp_timer_get_time() +
                            (int64_t) CONFIG_MIRILLA_NIGHT_MODE_GUARD_S * 1000000LL;

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
}
