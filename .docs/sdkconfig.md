# Configuración del sistema (sdkconfig.defaults)

> Comentarios originales de `sdkconfig.defaults`.

Proyecto: Mirilla electrónica — Waveshare ESP32-S3-CAM-OV5640 (ESP32-S3R8).

## CPU

`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y` — CPU a 240 MHz.

## Flash externa: 16MB

- Modo QIO, frecuencia 80M, tamaño 16MB.
- `CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y`.

## PSRAM Octal 8MB

> Comentario original: imprescindible para framebuffers a 1080p.

- `CONFIG_SPIRAM_MODE_OCT=y` (modo Octal), tipo auto, velocidad 80M.
- `CONFIG_SPIRAM_USE_MALLOC=y` con `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096`.
- `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`: buffers de Wi-Fi/lwIP en PSRAM.
- `CONFIG_SPIRAM_IGNORE_NOTFOUND=n`: si no hay PSRAM, fallar.

## Tareas

- Pila de la tarea main: 6144 bytes.
- Timeout del task watchdog: 10s.

## Wi-Fi

> Comentario original: buffers holgados para sostener subida continua de JPEG.

- TX estáticos: 16; TX dinámicos: 64; RX dinámicos: 64.
- Ventanas Block Ack (BA) TX/RX: 16.

## lwIP

> Comentario original: TCP orientado a throughput de subida.

- `CONFIG_LWIP_TCP_SND_BUF_DEFAULT=65535`
- `CONFIG_LWIP_TCP_WND_DEFAULT=65535`
- `CONFIG_LWIP_TCP_RECVMBOX_SIZE=64`

## esp32-camera

- `CONFIG_OV5640_SUPPORT=y`
- `CONFIG_SCCB_HARDWARE_I2C_PORT0=y`

## Log

- Nivel por defecto: INFO.
