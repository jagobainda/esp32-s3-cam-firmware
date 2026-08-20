#include "camera.h"

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

#if CONFIG_MIRILLA_DISABLE_NIGHT_MODE
    /*
     * El modo nocturno del OV5640 alarga el frame para exponer mas cuando hay
     * poca luz, y eso fija el frame rate en ~4 FPS aunque sobre ancho de banda
     * y pixel clock. Medido en hardware: con el activado el ritmo no pasa de
     * 3.9 FPS ni subiendo XCLK ni bajando la calidad JPEG.
     */
    if (sensor->set_aec2 != NULL && sensor->set_aec2(sensor, 0) == 0) {
        ESP_LOGI(TAG, "modo nocturno desactivado (ritmo constante sobre exposicion)");
    } else {
        ESP_LOGW(TAG, "no se pudo desactivar el modo nocturno");
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
