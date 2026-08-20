# esp32-s3-cam-firmware

Firmware en **C puro sobre ESP-IDF** para una **Waveshare ESP32-S3-CAM-OV5640**
(SKU 33699). Captura JPEG a 1080p y los sube por HTTP POST a un servidor de la LAN.

El servidor que los recibe vive en el repo **`esp32-s3-cam-to-server`**.

## Resultados medidos en hardware real

Placa por USB, red 2.4GHz, servidor en la misma LAN:

| Configuración | FPS real sostenido | JPEG medio | Caudal |
|---|---|---|---|
| **1920×1080, calidad 12, fb_count=3** | **5.17 FPS** (90 s, 465 frames) | 147 KB | 6.2 Mbps |
| 1920×1080, calidad 12, fb_count=2 | 3.92 FPS (60 s) | 281 KB | 9.0 Mbps |

Techo de captura pura del sensor, sin red de por medio (benchmark de arranque):
**5.73 FPS**, con el intervalo entre frames clavado en 169 ms.

El tamaño del JPEG depende muchísimo de la escena — medidos entre 53 KB y 281 KB
con calidad 12 — así que el caudal es consecuencia de eso, no un límite alcanzado:
el enlace Wi-Fi de esta placa da **25-28 Mbps** contra el servidor.

Ojo con una cosa: **el ritmo también depende de cómo responda el servidor**. Con una
respuesta HTTP que lleve cuerpo, uvicorn dispara un bloqueo de Nagle de ~200 ms por
petición y el ritmo cae a 3.9 FPS por mucho que la cámara y la red vayan sobradas.
Está explicado en detalle en el README de `esp32-s3-cam-to-server`.

## Hardware y pinout

ESP32-S3R8 (dual-core LX7 a 240 MHz, 8 MB PSRAM **Octal**, 16 MB flash), sensor
OV5640 de 5MP por DVP paralelo de 8 bits.

| Señal | GPIO | | Señal | GPIO |
|---|---|---|---|---|
| XCLK | 38 | | D3 | 46 |
| PCLK | 41 | | D4 | 42 |
| VSYNC | 17 | | D5 | 40 |
| HREF | 18 | | D6 | 39 |
| D0 | 45 | | D7 | 21 |
| D1 | 47 | | PWDN | no conectado |
| D2 | 48 | | RESET | no conectado |

SCCB (control del sensor) va por el **bus I2C compartido** de la placa, no uno
dedicado: **SCL=7, SDA=8** a 100 kHz.

Ese bus se inicializa *antes* de `esp_camera_init()` y se pasa por `sccb_i2c_port`,
dejando `pin_sccb_sda`/`pin_sccb_scl` en `GPIO_NUM_NC`. Detalle importante: con
ESP-IDF ≥ 5.4 el componente compila `sccb-ng.c`, que recupera el bus con
`i2c_master_get_bus_handle(port)`, así que hay que crearlo con la **API nueva**
(`i2c_new_master_bus`) y no con la legacy `driver/i2c.h`. Mezclarlas falla.

## Compilar y flashear

Requiere **ESP-IDF v5.5.1 o superior**. Depende de `espressif/esp32-camera` (^2.1.5,
resuelto a 2.1.7); no usa el BSP completo de Waveshare, que arrastra LVGL, audio y
pantalla que aquí no hacen falta.

```bash
idf.py set-target esp32s3
idf.py menuconfig          # menú "Mirilla electronica"
idf.py -p COM4 flash monitor
```

En el primer flasheo puede hacer falta mantener pulsado BOOT al conectar si
`idf.py flash` no detecta el puerto.

## Configuración

Todo lo ajustable está en `main/Kconfig.projbuild`, bajo **Mirilla electronica**:

| Opción | Por defecto | Notas |
|---|---|---|
| `MIRILLA_WIFI_SSID` / `_PASSWORD` | — | Solo 2.4GHz; el S3 no tiene 5GHz |
| `MIRILLA_WIFI_MAX_RETRY` / `_RETRY_DELAY_MS` | 10 / 5000 | Nunca abandona la reconexión |
| `MIRILLA_SERVER_HOST` / `_PORT` / `_PATH` | `192.168.1.133:8000/frame` | |
| `MIRILLA_API_KEY` | `mirilla-dev-key` | Va en la cabecera `X-API-Key` |
| `MIRILLA_FRAME_SIZE` | FHD 1920×1080 | Respaldos: HD 1280×720, SVGA 800×600 |
| `MIRILLA_JPEG_QUALITY` | 12 | 0-63, menor = más calidad y más bytes |
| `MIRILLA_FB_COUNT` | 3 | Con 2 la captura se serializa con la subida |
| `MIRILLA_XCLK_FREQ_HZ` | 20 MHz | Valor verificado por Waveshare |
| `MIRILLA_TARGET_FPS` | 5 | Techo del bucle, no una garantía |
| `MIRILLA_STATS_WINDOW_S` | 5 | Ventana del resumen de FPS |
| `MIRILLA_DISABLE_NIGHT_MODE` | sí | Prioriza ritmo constante sobre exposición |
| `MIRILLA_CAPTURE_BENCHMARK_S` | 6 | Mide la captura pura al arrancar |

PSRAM Octal viene ya activada en `sdkconfig.defaults` (`CONFIG_SPIRAM_MODE_OCT=y`);
sin ella los framebuffers de 1080p no caben y `esp_camera_init()` falla.

> Al **añadir una opción nueva** a `Kconfig.projbuild`, borra `sdkconfig` antes de
> compilar: confgen no reconcilia símbolos nuevos contra un `sdkconfig` existente y
> la compilación falla con `CONFIG_… undeclared`.

## Estructura

| Fichero | Responsabilidad |
|---|---|
| `main/main.c` | Arranque: NVS, comprobación de PSRAM, orquestación |
| `main/camera.c` | Bus I2C/SCCB, `camera_config_t` y benchmark de captura |
| `main/wifi.c` | Estación Wi-Fi con event group y tarea de reconexión |
| `main/uploader.c` | Bucle de captura → POST → estadística de FPS |

## Diagnóstico

El **benchmark de arranque** es lo primero que hay que mirar si el ritmo baja: mide
los FPS del sensor sin red de por medio, así que de un vistazo sabes si mirar a la
cámara o a la red.

```
I (7779) camera: benchmark: 5.73 FPS de captura pura (35 frames en 6.1s),
                 JPEG medio 97.1 KB, intervalo min 169 ms / max 169 ms
```

Después, cada frame se loguea con su tamaño, código HTTP y el desglose
captura/subida, y cada 5 s sale un resumen con el FPS real de la ventana:

```
I (9129) uploader: frame 0: 128173 bytes, HTTP 204, 0 ms captura + 62 ms subida
I (…)    uploader: == 5.2s | 5.20 FPS reales | 27 ok / 0 fallidos |
                   JPEG med 222.3 KB (min 215.2 / max 229.1) | 9.47 Mbps |
                   captura 1 ms + subida 114 ms
```

Si la captura se dispara por encima de ~1 ms, mira `MIRILLA_FB_COUNT`. Si lo que se
dispara es la subida, mira el servidor.
