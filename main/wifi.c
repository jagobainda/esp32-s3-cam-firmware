#include "wifi.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_RECONNECT_BIT BIT1

static EventGroupHandle_t s_wifi_events;
static int s_retry_count;

/*
 * La reconexion vive en su propia tarea y no en el handler de eventos: el
 * backoff necesita dormir, y dormir dentro del handler bloquearia el event
 * loop del sistema entero.
 */
static void wifi_reconnect_task(void *arg)
{
    (void) arg;

    for (;;) {
        xEventGroupWaitBits(s_wifi_events, WIFI_RECONNECT_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

        if (s_retry_count < CONFIG_MIRILLA_WIFI_MAX_RETRY) {
            s_retry_count++;
            ESP_LOGW(TAG, "reintento de conexion %d/%d",
                     s_retry_count, CONFIG_MIRILLA_WIFI_MAX_RETRY);
        } else {
            s_retry_count = 0;
            ESP_LOGW(TAG, "reintentos agotados, esperando %d ms",
                     CONFIG_MIRILLA_WIFI_RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(CONFIG_MIRILLA_WIFI_RETRY_DELAY_MS));
        }

        esp_wifi_connect();
    }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void) arg;
    (void) base;

    switch (id) {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "conectando a \"%s\"...", CONFIG_MIRILLA_WIFI_SSID);
        esp_wifi_connect();
        break;

    case WIFI_EVENT_STA_DISCONNECTED: {
        const wifi_event_sta_disconnected_t *ev = data;
        ESP_LOGW(TAG, "desconectado del AP (reason=%d)", ev->reason);
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_events, WIFI_RECONNECT_BIT);
        break;
    }

    default:
        break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void) arg;
    (void) base;

    if (id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;
        s_retry_count = 0;
        ESP_LOGI(TAG, "IP asignada: " IPSTR " (gw " IPSTR ")",
                 IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.gw));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t mirilla_wifi_start(void)
{
    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, on_ip_event, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    strlcpy((char *) wifi_config.sta.ssid, CONFIG_MIRILLA_WIFI_SSID,
            sizeof(wifi_config.sta.ssid));
    strlcpy((char *) wifi_config.sta.password, CONFIG_MIRILLA_WIFI_PASSWORD,
            sizeof(wifi_config.sta.password));

    if (strlen(CONFIG_MIRILLA_WIFI_PASSWORD) == 0) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /*
     * Subimos JPEG grandes de forma continua: sin ahorro de energia el
     * throughput es notablemente mas estable.
     */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    if (xTaskCreate(wifi_reconnect_task, "wifi_reconn", 3072, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    return ESP_OK;
}

bool mirilla_wifi_wait_connected(uint32_t timeout_ms)
{
    const TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    const EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, ticks);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

bool mirilla_wifi_is_connected(void)
{
    return (xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0;
}
