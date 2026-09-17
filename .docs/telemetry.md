# Telemetría (telemetry.c / telemetry.h)

## Diseño general

> Comentario original de `main/telemetry.h`.

Telemetría del microcontrolador, empaquetada en cabeceras HTTP que viajan
en el mismo POST que el frame.

No hay servidor HTTP en la placa a propósito: eso obligaría a una tarea y un
socket de escucha más, y a que alguien conociese la IP de la mirilla. Como ya
hay un POST por frame, colgar los datos de sus cabeceras sale gratis en
sockets, en tareas y en round-trips, y además llega correlado con el frame.

**Cuidado con el tamaño**: `esp_http_client` escribe las cabeceras en tandas de
`buffer_size_tx` bytes, una llamada a `esp_transport_write` por tanda. Si el
conjunto de cabeceras se pasa de ese buffer aparece un segundo write pequeño
y volvemos al escenario de Nagle documentado en el servidor (POST /frame).
Por eso las dos cadenas de aquí están acotadas y se comprueban al arrancar.

## Cabeceras

- `MIRILLA_TELEMETRY_HEADER_DEVICE` = `X-Mirilla-Device`
- `MIRILLA_TELEMETRY_HEADER_BOARD` = `X-Mirilla-Board`

> Comentario original: el servidor busca exactamente estas.

- `MIRILLA_TELEMETRY_HEADER_MAX = 288`: holgura suficiente para las dos cadenas con todos los campos presentes.
  El peor caso de `X-Mirilla-Device` ronda los 200 bytes con el estado del AEC
  incluido; con las dos cabeceras y el resto de la petición se anda por los
  520 bytes, muy por debajo de los 1024 de `buffer_size_tx` del cliente HTTP,
  que es la frontera que importa (ver el aviso del diseño general).

## Concurrencia

> Comentario original de `main/telemetry.c`.

El escritor y el lector son la misma tarea (uploader): publica las cifras al
cerrar la ventana estadística y las lee al componer la cabecera del
siguiente frame. Por eso no hay cerrojo.

## Rango del sensor de temperatura

> Comentario original de `main/telemetry.c`.

Rango de medida del sensor interno. El hardware tiene rangos predefinidos y
cruzar la frontera de dos de ellos hace que el driver rechace la
configuración, así que se elige de una lista cerrada en Kconfig.
Rangos disponibles: 50..125, 20..100, -10..80, -30..50, -40..20 °C.

## Inicialización (`mirilla_telemetry_init`)

> Comentario original de `main/telemetry.c`.

Que falle el sensor de temperatura no puede tumbar la subida de vídeo:
se avisa y el resto de la telemetría sigue viajando sin el campo.

## `mirilla_telemetry_device`

> Comentarios originales de `main/telemetry.c`.

Cadena con lo que cambia frame a frame: temperatura, RSSI, memoria libre,
uptime y el ritmo del bucle. Formato "clave=valor;clave=valor".

Notas del cuerpo:

- Fuera de rango, `temperature_sensor_get_celsius` devuelve `ESP_FAIL`: mejor omitir el campo que mentir.
- Estado del AEC. Es lo que explica un FPS bajo sin tocar nada más: el
  ritmo no puede pasar de 1000/frm, y si `exp` ha llegado a `expmax` con la
  ganancia arriba y `avg` por debajo del objetivo, el control está topado.
  Sin esto habría que abrir la puerta y enchufar el USB para verlo.
- `snprintf` trunca en silencio; si no cabe entero preferimos no mandar nada.

## `mirilla_telemetry_board`

> Comentarios originales de `main/telemetry.h` y `main/telemetry.c`.

Cadena con lo que no cambia: revisión del chip, memorias, versión del
firmware, motivo del último reinicio y configuración de captura. Se fija una
sola vez en el cliente HTTP y viaja en todos los POST sin coste de CPU.

- Se acotan versión e idf: vienen de git y pueden llegar a 32 bytes cada una.

## Cifras del bucle (`mirilla_telemetry_loop_t`)

> Comentarios originales de `main/telemetry.h`.

Cifras del bucle de captura que solo conoce el uploader.

| Campo | Significado |
|---|---|
| `fps` | FPS reales de la última ventana estadística |
| `capture_ms` | Media de captura en esa ventana |
| `upload_ms` | Media de subida en esa ventana |
| `frames_ok` | Acumulado desde el arranque |
| `frames_failed` | Acumulado desde el arranque |

## Funciones

- `mirilla_telemetry_init`: Prepara la telemetría: instala y habilita el sensor de temperatura interno
  y cachea los datos que no cambian durante la vida del programa. Devuelve
  `ESP_OK` aunque el sensor de temperatura falle: en ese caso el resto de la
  telemetría sigue funcionando y solo se omite el campo de temperatura.
- `mirilla_telemetry_set_loop`: Publica las cifras del bucle de captura para el siguiente muestreo.
  La llama el uploader al cerrar cada ventana estadística.
- `mirilla_telemetry_device`: Devuelve el número de bytes escritos (sin el terminador), 0 si no cabe.
- `mirilla_telemetry_board`: Ver arriba.
