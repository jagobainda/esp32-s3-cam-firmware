#include "telemetry.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

#include "camera.h"

#if CONFIG_MIRILLA_TELEMETRY_ENABLED
#include "driver/temperature_sensor.h"
#endif

static const char *TAG = "telemetry";

#if CONFIG_MIRILLA_TSENS_RANGE_50_125
#define TSENS_MIN 50
#define TSENS_MAX 125
#elif CONFIG_MIRILLA_TSENS_RANGE_20_100
#define TSENS_MIN 20
#define TSENS_MAX 100
#elif CONFIG_MIRILLA_TSENS_RANGE_M10_80
#define TSENS_MIN (-10)
#define TSENS_MAX 80
#elif CONFIG_MIRILLA_TSENS_RANGE_M30_50
#define TSENS_MIN (-30)
#define TSENS_MAX 50
#else
#define TSENS_MIN (-40)
#define TSENS_MAX 20
#endif

static mirilla_telemetry_loop_t s_loop;

#if CONFIG_MIRILLA_TELEMETRY_ENABLED
static temperature_sensor_handle_t s_tsens;
#endif

static const char *reset_reason_str(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:    return "poweron";
    case ESP_RST_EXT:        return "ext";
    case ESP_RST_SW:         return "sw";
    case ESP_RST_PANIC:      return "panic";
    case ESP_RST_INT_WDT:    return "int_wdt";
    case ESP_RST_TASK_WDT:   return "task_wdt";
    case ESP_RST_WDT:        return "wdt";
    case ESP_RST_DEEPSLEEP:  return "deepsleep";
    case ESP_RST_BROWNOUT:   return "brownout";
    case ESP_RST_SDIO:       return "sdio";
    case ESP_RST_USB:        return "usb";
    case ESP_RST_JTAG:       return "jtag";
    case ESP_RST_EFUSE:      return "efuse";
    case ESP_RST_PWR_GLITCH: return "pwr_glitch";
    case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
    default:                 return "unknown";
    }
}

esp_err_t mirilla_telemetry_init(void)
{
#if !CONFIG_MIRILLA_TELEMETRY_ENABLED
    ESP_LOGI(TAG, "telemetria desactivada en menuconfig");
    return ESP_OK;
#else
    const temperature_sensor_config_t config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(TSENS_MIN, TSENS_MAX);

    esp_err_t err = temperature_sensor_install(&config, &s_tsens);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sensor de temperatura no instalado (%s), se omitira",
                 esp_err_to_name(err));
        s_tsens = NULL;
        return ESP_OK;
    }

    err = temperature_sensor_enable(s_tsens);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sensor de temperatura no habilitado (%s), se omitira",
                 esp_err_to_name(err));
        temperature_sensor_uninstall(s_tsens);
        s_tsens = NULL;
        return ESP_OK;
    }

    ESP_LOGI(TAG, "sensor de temperatura activo en el rango %d..%d C",
             TSENS_MIN, TSENS_MAX);
    return ESP_OK;
#endif
}

void mirilla_telemetry_set_loop(const mirilla_telemetry_loop_t *loop)
{
    if (loop != NULL) {
        s_loop = *loop;
    }
}

size_t mirilla_telemetry_device(char *out, size_t len)
{
#if !CONFIG_MIRILLA_TELEMETRY_ENABLED
    (void) out;
    (void) len;
    return 0;
#else
    if (out == NULL || len == 0) {
        return 0;
    }

    int written = 0;

    if (s_tsens != NULL) {
        float celsius = 0.0f;
        if (temperature_sensor_get_celsius(s_tsens, &celsius) == ESP_OK) {
            written += snprintf(out + written, len - written, "t=%.1f;", celsius);
        }
    }

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        written += snprintf(out + written, len - written, "rssi=%d;ch=%u;",
                            ap.rssi, (unsigned) ap.primary);
    }

    mirilla_camera_aec_t aec;
    mirilla_camera_aec(&aec);
    if (aec.valid) {
        written += snprintf(out + written, len - written,
                            "exp=%u;expmax=%u;frm=%u;gain=%u.%02u;avg=%u;",
                            aec.exposure_ms, aec.ceiling_ms, aec.frame_ms,
                            aec.gain_x100 / 100, aec.gain_x100 % 100, aec.avg);
    }

    written += snprintf(
        out + written, len - written,
        "heap=%" PRIu32 ";heapmin=%" PRIu32 ";psram=%u;up=%lld;"
        "fps=%.2f;cap=%" PRIu32 ";upl=%" PRIu32 ";ok=%" PRIu32 ";ko=%" PRIu32,
        esp_get_free_internal_heap_size(),
        esp_get_minimum_free_heap_size(),
        (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        esp_timer_get_time() / 1000000LL,
        s_loop.fps, s_loop.capture_ms, s_loop.upload_ms,
        s_loop.frames_ok, s_loop.frames_failed);

    if (written < 0 || (size_t) written >= len) {
        ESP_LOGW(TAG, "cabecera de telemetria truncada (%d bytes), se omite", written);
        return 0;
    }
    return (size_t) written;
#endif
}

size_t mirilla_telemetry_board(char *out, size_t len)
{
#if !CONFIG_MIRILLA_TELEMETRY_ENABLED
    (void) out;
    (void) len;
    return 0;
#else
    if (out == NULL || len == 0) {
        return 0;
    }

    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_bytes = 0;
    if (esp_flash_get_size(NULL, &flash_bytes) != ESP_OK) {
        flash_bytes = 0;
    }

    const esp_app_desc_t *app = esp_app_get_description();

    const int written = snprintf(
        out, len,
        "rev=v%d.%d;cores=%d;psram=%u;flash=%" PRIu32 ";"
        "fw=%.16s;built=%.11s;idf=%.16s;rst=%s;res=%s;q=%d;tfps=%d",
        chip.revision / 100, chip.revision % 100, chip.cores,
        (unsigned) esp_psram_get_size(), flash_bytes,
        app->version, app->date, app->idf_ver,
        reset_reason_str(), mirilla_camera_resolution_str(),
        CONFIG_MIRILLA_JPEG_QUALITY, CONFIG_MIRILLA_TARGET_FPS);

    if (written < 0 || (size_t) written >= len) {
        ESP_LOGW(TAG, "cabecera de placa truncada (%d bytes), se omite", written);
        return 0;
    }
    return (size_t) written;
#endif
}
