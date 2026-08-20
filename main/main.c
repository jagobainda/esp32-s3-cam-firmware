/*
 * Mirilla electronica - firmware de captura y subida.
 *
 * Placa: Waveshare ESP32-S3-CAM-OV5640 (ESP32-S3R8, 8MB PSRAM Octal).
 * Captura JPEG por DVP y lo sube por HTTP POST a un servidor de la LAN.
 */

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "camera.h"
#include "uploader.h"
#include "wifi.h"

static const char *TAG = "mirilla";

static esp_err_t check_psram(void)
{
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM NO inicializada: los framebuffers a 1080p no caben. "
                      "Revisa 'ESP PSRAM -> Octal Mode PSRAM' en menuconfig.");
        return ESP_ERR_NOT_FOUND;
    }

    const size_t total = esp_psram_get_size();
    ESP_LOGI(TAG, "PSRAM detectada: %u bytes totales (%u KB libres)",
             (unsigned) total,
             (unsigned) (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    return ESP_OK;
}

static void log_chip_info(void)
{
    esp_chip_info_t info;
    esp_chip_info(&info);
    ESP_LOGI(TAG, "ESP32-S3 rev v%d.%d, %d nucleos, heap interno libre %" PRIu32 " bytes",
             info.revision / 100, info.revision % 100, info.cores,
             esp_get_free_heap_size());
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Mirilla electronica: arrancando ===");
    log_chip_info();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(check_psram());
    ESP_ERROR_CHECK(mirilla_camera_init());
    mirilla_camera_benchmark(CONFIG_MIRILLA_CAPTURE_BENCHMARK_S);

    ESP_ERROR_CHECK(mirilla_wifi_start());
    ESP_ERROR_CHECK(mirilla_uploader_start());

    ESP_LOGI(TAG, "inicializacion completa, la tarea de subida toma el relevo");
}
