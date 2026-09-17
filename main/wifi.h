#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t mirilla_wifi_start(void);

bool mirilla_wifi_wait_connected(uint32_t timeout_ms);

bool mirilla_wifi_is_connected(void);
