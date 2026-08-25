#include "uploader.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "camera.h"
#include "wifi.h"

static const char *TAG = "uploader";

#define FRAME_PERIOD_US (1000000LL / CONFIG_MIRILLA_TARGET_FPS)
#define STATS_WINDOW_US ((int64_t) CONFIG_MIRILLA_STATS_WINDOW_S * 1000000LL)

/* Estadistica acumulada dentro de la ventana movil. */
typedef struct {
    int64_t window_start_us;
    uint32_t frames_ok;
    uint32_t frames_failed;
    uint64_t bytes;
    uint32_t bytes_min;
    uint32_t bytes_max;
    uint64_t capture_us;
    uint64_t upload_us;
} stats_t;

static void stats_reset(stats_t *s, int64_t now_us)
{
    memset(s, 0, sizeof(*s));
    s->window_start_us = now_us;
    s->bytes_min = UINT32_MAX;
}

static void stats_report(const stats_t *s, int64_t now_us)
{
    const double elapsed_s = (double) (now_us - s->window_start_us) / 1000000.0;
    if (elapsed_s <= 0.0) {
        return;
    }

    const uint32_t total = s->frames_ok + s->frames_failed;
    if (total == 0) {
        ESP_LOGW(TAG, "ventana de %.1fs sin ningun frame", elapsed_s);
        return;
    }

    const double fps = s->frames_ok / elapsed_s;
    const double mbps = (double) s->bytes * 8.0 / elapsed_s / 1000000.0;

    ESP_LOGI(TAG,
             "== %.1fs | %.2f FPS reales | %u ok / %u fallidos | "
             "JPEG med %.1f KB (min %.1f / max %.1f) | %.2f Mbps | "
             "captura %.0f ms + subida %.0f ms",
             elapsed_s, fps, s->frames_ok, s->frames_failed,
             s->frames_ok ? (double) s->bytes / s->frames_ok / 1024.0 : 0.0,
             s->bytes_min == UINT32_MAX ? 0.0 : s->bytes_min / 1024.0,
             s->bytes_max / 1024.0,
             mbps,
             s->frames_ok ? (double) s->capture_us / s->frames_ok / 1000.0 : 0.0,
             s->frames_ok ? (double) s->upload_us / s->frames_ok / 1000.0 : 0.0);
}

/* Descarta el cuerpo de la respuesta; solo nos interesa el codigo de estado. */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    (void) evt;
    return ESP_OK;
}

static esp_http_client_handle_t make_client(void)
{
    char url[160];
    snprintf(url, sizeof(url), "http://%s:%d%s",
             CONFIG_MIRILLA_SERVER_HOST,
             CONFIG_MIRILLA_SERVER_PORT,
             CONFIG_MIRILLA_SERVER_PATH);

    const esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = CONFIG_MIRILLA_HTTP_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .keep_alive_enable = true,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "no se pudo crear el cliente HTTP para %s", url);
        return NULL;
    }

    esp_http_client_set_header(client, "Content-Type", "image/jpeg");
    esp_http_client_set_header(client, "X-API-Key", CONFIG_MIRILLA_API_KEY);

    ESP_LOGI(TAG, "destino: %s", url);
    return client;
}

static void uploader_task(void *arg)
{
    (void) arg;

    ESP_LOGI(TAG, "esperando conectividad antes de empezar a capturar");
    mirilla_wifi_wait_connected(0);

    esp_http_client_handle_t client = make_client();
    if (client == NULL) {
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "arrancando bucle a %d FPS objetivo (%s, calidad %d)",
             CONFIG_MIRILLA_TARGET_FPS, mirilla_camera_resolution_str(),
             CONFIG_MIRILLA_JPEG_QUALITY);

    stats_t stats;
    stats_reset(&stats, esp_timer_get_time());
    uint32_t frame_index = 0;

    for (;;) {
        const int64_t cycle_start_us = esp_timer_get_time();

        if (!mirilla_wifi_is_connected()) {
            ESP_LOGW(TAG, "sin Wi-Fi, esperando reconexion");
            mirilla_wifi_wait_connected(0);
            /* La conexion TCP anterior ya no sirve tras caerse el enlace. */
            esp_http_client_close(client);
            stats_reset(&stats, esp_timer_get_time());
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        const int64_t captured_us = esp_timer_get_time();
        if (fb == NULL) {
            ESP_LOGE(TAG, "esp_camera_fb_get devolvio NULL");
            stats.frames_failed++;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        esp_http_client_set_post_field(client, (const char *) fb->buf, fb->len);
        const esp_err_t err = esp_http_client_perform(client);
        const int64_t uploaded_us = esp_timer_get_time();

        const uint32_t len = fb->len;
        const int status = (err == ESP_OK) ? esp_http_client_get_status_code(client) : -1;

        esp_camera_fb_return(fb);

        if (err == ESP_OK && status >= 200 && status < 300) {
            stats.frames_ok++;
            stats.bytes += len;
            if (len < stats.bytes_min) stats.bytes_min = len;
            if (len > stats.bytes_max) stats.bytes_max = len;
            stats.capture_us += (uint64_t) (captured_us - cycle_start_us);
            stats.upload_us += (uint64_t) (uploaded_us - captured_us);

            ESP_LOGI(TAG, "frame %lu: %lu bytes, HTTP %d, %lld ms captura + %lld ms subida",
                     (unsigned long) frame_index, (unsigned long) len, status,
                     (captured_us - cycle_start_us) / 1000,
                     (uploaded_us - captured_us) / 1000);
        } else {
            stats.frames_failed++;
            ESP_LOGE(TAG, "frame %lu: %lu bytes, fallo de subida (%s, HTTP %d)",
                     (unsigned long) frame_index, (unsigned long) len,
                     esp_err_to_name(err), status);
            /* Fuerza reapertura de socket en el siguiente intento. */
            esp_http_client_close(client);
        }

        frame_index++;

        int64_t now_us = esp_timer_get_time();
        if (now_us - stats.window_start_us >= STATS_WINDOW_US) {
            stats_report(&stats, now_us);
            stats_reset(&stats, now_us);
        }

        /* Al final del ciclo, no entre captura y subida: si toca revisar la
           exposicion, el I2C se come tiempo de espera y no de latencia. */
        mirilla_camera_keep_exposure_policy();

        /* Ritmo objetivo: dormimos solo lo que sobre del periodo. */
        const int64_t spent_us = esp_timer_get_time() - cycle_start_us;
        if (spent_us < FRAME_PERIOD_US) {
            vTaskDelay(pdMS_TO_TICKS((FRAME_PERIOD_US - spent_us) / 1000));
        } else {
            /* Cede CPU aunque vayamos tarde, para no matar de hambre a otras tareas. */
            vTaskDelay(1);
        }
    }
}

esp_err_t mirilla_uploader_start(void)
{
    /* Pila holgada: esp_http_client + mbedtls-free path aun asi usa bastante. */
    if (xTaskCreate(uploader_task, "uploader", 6144, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
