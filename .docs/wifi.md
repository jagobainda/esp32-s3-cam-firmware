# Wi-Fi (wifi.c / wifi.h)

## API pública (wifi.h)

- `mirilla_wifi_start`: Arranca la pila de red y la conexión en modo estación. No bloquea: la
  reconexión se gestiona por eventos durante toda la vida del programa.
- `mirilla_wifi_wait_connected`: Bloquea hasta que hay IP asignada. Con `timeout_ms = 0` espera indefinidamente.
  Devuelve true si hay conectividad.
- `mirilla_wifi_is_connected`: true si ahora mismo hay IP asignada.

## Reconexión

> Comentario original de `main/wifi.c`.

La reconexión vive en su propia tarea y no en el handler de eventos: el
backoff necesita dormir, y dormir dentro del handler bloquearía el event
loop del sistema entero.

Comportamiento de `wifi_reconnect_task`:

- Mientras queden intentos (`CONFIG_MIRILLA_WIFI_MAX_RETRY`), reconecta de inmediato.
- Agotados los intentos, espera `CONFIG_MIRILLA_WIFI_RETRY_DELAY_MS` y reinicia el contador.

## Eventos

- `WIFI_EVENT_STA_START`: conecta al SSID configurado.
- `WIFI_EVENT_STA_DISCONNECTED`: registra el motivo, limpia el bit de conectado y señala el bit de reconexión.
- `IP_EVENT_STA_GOT_IP`: registra IP y gateway, resetea el contador de reintentos y marca conectado.

## Configuración

> Comentario original de `main/wifi.c`.

Subimos JPEG grandes de forma continua: sin ahorro de energía (`WIFI_PS_NONE`) el
throughput es notablemente más estable.

Otros detalles:

- Autenticación mínima WPA2; si la contraseña está vacía, red abierta.
- PMF capaz pero no requerido.
