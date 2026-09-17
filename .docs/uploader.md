# Uploader (uploader.c / uploader.h)

## API pública (uploader.h)

- `mirilla_uploader_start`: Lanza la tarea que captura frames y los sube al servidor por HTTP POST.
  Requiere que la cámara esté inicializada y el Wi-Fi arrancado.

## Estructura interna

### Estadísticas de la ventana móvil (`stats_t`)

> Comentario original de `main/uploader.c`.

Estadística acumulada dentro de la ventana móvil (duración: `MIRILLA_STATS_WINDOW_S`).

### Totales acumulados

> Comentario original de `main/uploader.c`.

`s_total_ok` / `s_total_failed`: acumulados de toda la vida del programa, no de la ventana: la ventana se
reinicia cada `MIRILLA_STATS_WINDOW_S` y el servidor quiere el total.

## Cliente HTTP (`make_client`)

> Comentarios originales de `main/uploader.c`.

- Los datos de placa no cambian nunca, así que basta fijar la cabecera una
  vez: el cliente conserva la lista entre peticiones y la reenvía en todas
  sin volver a gastar CPU en componerla.
- `http_event_handler`: descarta el cuerpo de la respuesta; solo interesa el código de estado.

## Bucle de la tarea (`uploader_task`)

> Comentarios originales de `main/uploader.c`.

- Al arrancar: espera conectividad antes de empezar a capturar.
- Telemetría: se compone justo antes del envío, para que el dato sea lo más fresco posible.
  La latencia de subida es la excepción: describe la ventana anterior,
  porque la de este frame todavía no ha ocurrido.
- Tras caerse el enlace Wi-Fi: la conexión TCP anterior ya no sirve, se cierra el cliente y se reinicia la ventana.
- En caso de fallo de subida: fuerza reapertura de socket en el siguiente intento (`esp_http_client_close`).
- Los mismos números que acaban de ir al log viajan al servidor en la
  cabecera del siguiente frame, que es donde alguien los va a ver.
- `mirilla_camera_keep_exposure_policy` se llama al final del ciclo, no entre captura y subida: si toca revisar la
  exposición, el I2C se come tiempo de espera y no de latencia.
- Ritmo objetivo: se duerme solo lo que sobre del periodo. Si se llega tarde,
  se cede CPU de todos modos, para no matar de hambre a otras tareas.

## Arranque (`mirilla_uploader_start`)

> Comentario original de `main/uploader.c`.

Pila holgada: `esp_http_client` + mbedtls-free path aun así usa bastante (pila de 6144 bytes).
