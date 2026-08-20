#pragma once

#include "esp_err.h"

/**
 * Lanza la tarea que captura frames y los sube al servidor por HTTP POST.
 * Requiere que la camara este inicializada y el Wi-Fi arrancado.
 */
esp_err_t mirilla_uploader_start(void);
