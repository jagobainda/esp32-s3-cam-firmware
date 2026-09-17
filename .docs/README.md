# Mirilla electrónica — firmware de captura y subida

> Comentario original de `main/main.c` (cabecera del proyecto).

Placa: Waveshare ESP32-S3-CAM-OV5640 (ESP32-S3R8, 8MB PSRAM Octal).
Captura JPEG por DVP y lo sube por HTTP POST a un servidor de la LAN.

## Índice

| Archivo | Contenido |
|---|---|
| [camera.md](camera.md) | Pinout DVP, política de exposición del OV5640, inicialización, benchmark y API pública |
| [telemetry.md](telemetry.md) | Telemetría en cabeceras HTTP, sensor de temperatura y campos que viajan en cada POST |
| [uploader.md](uploader.md) | Bucle de captura y subida HTTP, ventanas estadísticas y ritmo |
| [wifi.md](wifi.md) | Conexión en modo estación, reconexión con backoff y ahorro de energía |
| [sdkconfig.md](sdkconfig.md) | Decisiones de configuración del sistema (CPU, flash, PSRAM, Wi-Fi, lwIP) |

## Arranque (`app_main`)

> Comentarios originales de `main/main.c`.

- PSRAM NO inicializada: los framebuffers a 1080p no caben. Revisar "ESP PSRAM → Octal Mode PSRAM" en menuconfig (`check_psram`).
- La inicialización completa deja el relevo a la tarea de subida: NVS → PSRAM → cámara (con benchmark) → telemetría → Wi-Fi → uploader.
