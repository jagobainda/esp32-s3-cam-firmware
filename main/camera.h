#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_camera.h"

/**
 * Ultimo estado del AEC/AGC leido del sensor por la vigilancia periodica.
 *
 * `frame_ms` es el dato que importa para el ritmo: es la duracion real del
 * frame ((VTS + lineas de relleno) x reloj de linea). A 1080p el driver DVP
 * entrega un frame de cada dos, asi que el techo del aparato es
 * 1000/(2 x frame_ms) FPS: con los 84.63ms del modo dia salen 5.9. Si
 * frame_ms sube sin que nadie haya tocado la configuracion, es el AEC
 * alargando el frame, y cada milisegundo cuesta el doble.
 *
 * El resto distingue "no hay luz" de "el control esta topado": con
 * exposure_ms == ceiling_ms, la ganancia en su techo y `avg` todavia por
 * debajo de MIRILLA_AEC_TARGET, el AEC ha hecho lo que ha podido.
 */
typedef struct {
    bool valid;           /* false mientras no se haya podido leer por SCCB */
    uint16_t exposure_ms; /* exposicion elegida por el AEC */
    uint16_t ceiling_ms;  /* techo que se le ha impuesto */
    uint16_t frame_ms;    /* duracion real del frame */
    uint16_t gain_x100;   /* ganancia analogica, x100 (1250 = 12.5x) */
    uint8_t avg;          /* media medida en la ventana del AEC, 0-255 */
} mirilla_camera_aec_t;

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

/**
 * Copia el ultimo estado del AEC muestreado. Se refresca en cada revision de
 * la politica de exposicion, o sea cada MIRILLA_EXPOSURE_GUARD_S segundos.
 */
void mirilla_camera_aec(mirilla_camera_aec_t *out);
