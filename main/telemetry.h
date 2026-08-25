#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Telemetria del microcontrolador, empaquetada en cabeceras HTTP que viajan
 * en el mismo POST que el frame.
 *
 * No hay servidor HTTP en la placa a proposito: eso obligaria a una tarea y un
 * socket de escucha mas, y a que alguien conociese la IP de la mirilla. Como ya
 * hay un POST por frame, colgar los datos de sus cabeceras sale gratis en
 * sockets, en tareas y en round-trips, y ademas llega correlado con el frame.
 *
 * Cuidado con el tamano: esp_http_client escribe las cabeceras en tandas de
 * `buffer_size_tx` bytes, una llamada a esp_transport_write por tanda. Si el
 * conjunto de cabeceras se pasa de ese buffer aparece un segundo write pequeno
 * y volvemos al escenario de Nagle documentado en el servidor (POST /frame).
 * Por eso las dos cadenas de aqui estan acotadas y se comprueban al arrancar.
 */

/** Nombre de las cabeceras. El servidor busca exactamente estas. */
#define MIRILLA_TELEMETRY_HEADER_DEVICE "X-Mirilla-Device"
#define MIRILLA_TELEMETRY_HEADER_BOARD  "X-Mirilla-Board"

/** Holgura suficiente para las dos cadenas con todos los campos presentes. */
#define MIRILLA_TELEMETRY_HEADER_MAX 224

/* Cifras del bucle de captura que solo conoce el uploader. */
typedef struct {
    float fps;              /* FPS reales de la ultima ventana estadistica */
    uint32_t capture_ms;    /* Media de captura en esa ventana */
    uint32_t upload_ms;     /* Media de subida en esa ventana */
    uint32_t frames_ok;     /* Acumulado desde el arranque */
    uint32_t frames_failed; /* Acumulado desde el arranque */
} mirilla_telemetry_loop_t;

/**
 * Prepara la telemetria: instala y habilita el sensor de temperatura interno
 * y cachea los datos que no cambian durante la vida del programa.
 *
 * Devuelve ESP_OK aunque el sensor de temperatura falle: en ese caso el resto
 * de la telemetria sigue funcionando y solo se omite el campo de temperatura.
 */
esp_err_t mirilla_telemetry_init(void);

/**
 * Publica las cifras del bucle de captura para el siguiente muestreo.
 * La llama el uploader al cerrar cada ventana estadistica.
 */
void mirilla_telemetry_set_loop(const mirilla_telemetry_loop_t *loop);

/**
 * Cadena con lo que cambia frame a frame: temperatura, RSSI, memoria libre,
 * uptime y el ritmo del bucle. Formato "clave=valor;clave=valor".
 * @return numero de bytes escritos (sin el terminador), 0 si no cabe.
 */
size_t mirilla_telemetry_device(char *out, size_t len);

/**
 * Cadena con lo que no cambia: revision del chip, memorias, version del
 * firmware, motivo del ultimo reinicio y configuracion de captura. Se fija una
 * sola vez en el cliente HTTP y viaja en todos los POST sin coste de CPU.
 */
size_t mirilla_telemetry_board(char *out, size_t len);
