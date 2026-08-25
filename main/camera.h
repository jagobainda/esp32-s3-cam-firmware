#pragma once

#include "esp_err.h"
#include "esp_camera.h"

/**
 * Inicializa el bus I2C compartido (SCCB) y a continuacion el sensor OV5640
 * por DVP. Debe llamarse una sola vez, antes de cualquier captura.
 */
esp_err_t mirilla_camera_init(void);

/** Descripcion legible de la resolucion configurada, p.ej. "1920x1080". */
const char *mirilla_camera_resolution_str(void);

/**
 * Reafirma que el bloque AEC/AGC del sensor sigue con la politica de
 * exposicion configurada (modo dia o noche, techos de exposicion y ganancia,
 * objetivo y ventana de medicion). Pensada para llamarse desde el bucle de
 * captura en cada frame: internamente solo toca el bus SCCB una vez cada
 * MIRILLA_EXPOSURE_GUARD_S segundos, y en el caso normal la comprobacion son
 * lecturas de registro sin escritura.
 */
void mirilla_camera_keep_exposure_policy(void);

/**
 * Captura en bucle durante `seconds` sin subir nada y registra el ritmo puro
 * del sensor. Permite separar el limite de la camara del limite de la red.
 */
void mirilla_camera_benchmark(int seconds);
