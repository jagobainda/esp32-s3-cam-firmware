#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * Arranca la pila de red y la conexion en modo estacion. No bloquea: la
 * reconexion se gestiona por eventos durante toda la vida del programa.
 */
esp_err_t mirilla_wifi_start(void);

/**
 * Bloquea hasta que hay IP asignada.
 * @param timeout_ms  0 = esperar indefinidamente.
 * @return true si hay conectividad.
 */
bool mirilla_wifi_wait_connected(uint32_t timeout_ms);

/** true si ahora mismo hay IP asignada. */
bool mirilla_wifi_is_connected(void);
