# Cámara (camera.c / camera.h)

## Pinout DVP

> Comentario original de `main/camera.c` (bloque de pines).

Pinout DVP de la Waveshare ESP32-S3-CAM-OV5640 (SKU 33699).
Contrastado entre los ejemplos oficiales `04_dvp_camera_display` y
`05_lvgl_brookesia`, que coinciden pin a pin.

Pines resultantes: XCLK 38, PCLK 41, VSYNC 17, HREF 18,
D0-D7 → 45, 47, 48, 46, 42, 40, 39, 21.

- PWDN y RESET no están cableados en esta variante de la placa (GPIO_NUM_NC).
- SCCB: bus I2C compartido con el resto de periféricos, no uno dedicado (I2C_NUM_0, SCL=GPIO7, SDA=GPIO8).

## Política de exposición del OV5640

> Comentario original extenso de `main/camera.c`.

El sensor arranca con el modo nocturno activado (0x3a00 bit 2) y con los
límites del AEC que trae la tabla por defecto del driver esp32-camera,
calculados para otro modo de video. En 1080p JPEG con XCLK de 20MHz el modo
que corre de verdad es este:

```
SYSCLK 50MHz, HTS 2844, VTS 1488 -> linea 56.9us, frame 84.6ms (11.8 FPS)
```

y los límites que quedan puestos, estos:

```
techo de exposicion (0x3a02/03 y 0x3a14/15) = 0x03d8 = 984 lineas = 56ms
techo de ganancia   (0x3a18/19)             = 0x00f8 = 15.5x
pasos de banda      (0x3a08..0x3a0b)        = 295 y 246 lineas, cuando los
                                              reales aqui son 175 (50Hz)
                                              y 146 (60Hz)
```

O sea: el AEC no puede pasar de 56ms x 15.5x aunque el frame dure 84.6ms, y
encima cuantiza la exposición con pasos de banda de otro modo. En la mirilla
eso pesa más de lo que parece, porque la óptica solo proyecta un círculo que
ocupa ~10% del frame y el resto es negro: la media que mide el AEC no puede
alcanzar su objetivo por mucho que exponga, así que el control se queda
pegado al techo y la imagen es, literalmente, "lo que da el techo".

Aquí se reprograman los límites para el modo que está corriendo (fórmulas
del datasheet, las mismas que usa `drivers/media/i2c/ov5640.c` en Linux) y se
elige qué hace el sensor cuando se queda sin luz:

- **MODO DÍA** — modo nocturno off, el frame nunca se alarga: dura siempre
  HTS x VTS. El ritmo es fijo pase lo que pase. Es el default.
- **MODO NOCHE** — modo nocturno on: el AEC alarga el frame con líneas de
  relleno para exponer más tiempo. Cuesta FPS de verdad.

### Coste del modo noche

> Comentario original (corrección histórica): este comentario decía lo contrario y costó 2.5 FPS.

El techo de exposición NO es un suelo de FPS.

1. A 1080p el driver DVP entrega un frame de cada DOS del sensor. Medido
   con el benchmark de captura pura en esta placa: frame de 84.63ms
   (11.81 FPS de sensor) y llegan 169ms clavados, min = max, 5.81 FPS.
   Alargar el frame del sensor 1ms cuesta 2ms de periodo.
2. El bucle de subida es secuencial (esperar frame, subirlo), así que
   encima el periodo es entrega + subida, no el mayor de los dos.

De ahí salieron los 2.5 FPS: con el techo en 150ms el sensor se iba a ~200ms
de frame, el DVP lo doblaba a 400ms y el bucle quedaba clavado en 2.5. El
suelo real del modo noche no es `1000/MAX_EXPOSURE_MS`, es la mitad.

Por eso el techo se recorta al periodo del FPS objetivo dividido por esa
relación, y por eso la duración real del frame se lee del sensor y viaja en
la telemetría: con la placa montada en la puerta no hay USB, y sin ese dato
esto se manifiesta como un FPS bajo sin causa aparente.

### Registros implicados

> Comentario original: todos del datasheet del OV5640.

| Registro | Función |
|---|---|
| 0x3034..0x3037, 0x3108 | divisores del PLL, para calcular SYSCLK |
| 0x3808/0x380a | tamaño de salida (para la ventana de medición) |
| 0x380c/0x380e | HTS / VTS del modo actual |
| 0x3500..0x3502 | exposición en líneas (20 bits, 4 de fracción) |
| 0x350a/0x350b | ganancia real, formato 6.4 (16 unidades = 1x) |
| 0x350c/0x350d | líneas de relleno que añade el modo nocturno |
| 0x3a00 | bit 2 modo nocturno, bit 5 filtro de banda |
| 0x3a02/03 y 0x3a14/15 | techo de exposición (60Hz y 50Hz) |
| 0x3a08..0x3a0b | paso de banda de 50Hz y de 60Hz |
| 0x3a0d/0x3a0e | número máximo de bandas (60Hz y 50Hz) |
| 0x3a0f,10,11,1b,1e,1f | nivel medio objetivo del AEC y sus márgenes |
| 0x3a18/0x3a19 | techo de ganancia |
| 0x5680..0x5687 | ventana sobre la que el AEC promedia |
| 0x56a1 | media medida (solo lectura, para el diagnóstico) |

## Constantes y macros

- El techo de exposición y el VTS son campos de 12 bits en el sensor (`OV5640_MAX_EXPOSURE_LINES = 0x0fff`).
- `MIRILLA_SENSOR_FRAMES_PER_CAPTURE = 2`: frames del sensor por cada frame que entrega el driver DVP. A 1080p es 2, medido: el sensor va a 84.63ms y `esp_camera_fb_get()` devuelve uno cada 169ms, sin dispersión. Es lo que convierte la duración del frame en el techo de FPS del aparato entero.
- Ganancia real en formato 6.4: 16 unidades = 1x, tope del registro 0x3ff (`OV5640_GAIN_UNITS_PER_X`, `OV5640_MAX_GAIN_UNITS`).

## Funciones internas

### `reg_ensure`

> Comentario original de `main/camera.c`.

Escribe solo si hace falta y relee para confirmar: una escritura SCCB
perdida no debe pasar inadvertida. Devuelve false si no se pudo dejar el
registro como se pedía, y marca `fixed` si hubo que tocarlo (al arrancar es
lo normal; más tarde significa que algo lo había cambiado).

### `exposure_sysclk_hz`

> Comentario original de `main/camera.c`.

SYSCLK a partir de los divisores del PLL, tal cual lo documenta OmniVision.
De él salen el reloj de línea, el techo de exposición en líneas y los pasos
del filtro de banda; sin calcularlo hay que adivinarlos, que es justo lo que
hace la tabla por defecto del driver.

Detalles de implementación:

- `0x3034[3:0]`: 8 y 10 son bits por pixel y dividen; el resto no.
- Se calcula en unidades de 10kHz para que el VCO no desborde 32 bits.

### `exposure_mode_read`

> Comentario original de `main/camera.c`.

Geometría y reloj del modo actual, leídos del propio sensor.

Sobre el techo de exposición: en modo día no tiene sentido pedir más de un
frame: el sensor no puede exponer más de VTS-4 líneas sin alargarlo, y
alargarlo es precisamente lo que hace el modo nocturno.

En modo noche el recorte es el periodo del FPS objetivo repartido entre
los frames de sensor que cuesta cada captura. No garantiza ese ritmo (la
subida va detrás, en el mismo bucle), pero impide lo que no tiene
defensa posible: pedir una exposición que por sí sola ya se pasa del
periodo objetivo.

### `exposure_lines_to_us`

Duración en microsegundos de un número de líneas del modo actual.

### `exposure_policy_apply`

> Comentario original de `main/camera.c`.

Deja el bloque AEC/AGC como pide la configuración. Idempotente a propósito:
se llama al arrancar y luego cada `MIRILLA_EXPOSURE_GUARD_S` segundos, y en el
caso normal son solo lecturas.

Notas del cuerpo de la función:

- Techo de exposición, en las dos variantes (60Hz y 50Hz) que mira el AEC.
- Filtro de banda: cuantiza la exposición en múltiplos de medio ciclo de
  la red para que la luz artificial no deje bandas horizontales. El paso
  depende del reloj de línea, así que hay que recalcularlo para este modo,
  y el número máximo de bandas es lo que limita la exposición mientras el
  filtro está activo.
- Techo de ganancia analógica, en formato 6.4.
- Nivel medio objetivo del AEC y su histeresis, con las proporciones que
  documenta OmniVision: ±8% de zona estable, y el doble y la mitad para
  decidir cuándo corregir de golpe en vez de poco a poco.
- Ventana de medición: el AEC promedia esta región y la compara con el
  objetivo; por defecto abarca el frame entero, y en la mirilla eso es
  medir sobre todo el negro que rodea al círculo de la óptica. Un cuadrado
  centrado del tamaño del círculo hace que el AEC regule por la escena de
  verdad en vez de quedarse pegado al techo.
- Y al final el modo nocturno, que es quien decide si el frame se alarga.
  Sin modo nocturno nadie debería estar añadiendo líneas de relleno (se fuerza VTS_EXTRA a 0).
- Si todo fue bien, el status del driver no debe contradecir a los registros (`sensor->status.aec2`).

### `exposure_sample_state`

> Comentario original de `main/camera.c`.

Qué ha elegido el AEC ahora mismo. Es el diagnóstico que dice si la escena
sale oscura porque no hay luz o porque el control está topado: si exposición
y ganancia están en su techo y la media medida sigue por debajo del
objetivo, es lo segundo y hay margen subiendo los techos.

`frame_ms` es además el único sitio donde se ve la duración real del frame,
que es lo que fija el techo de FPS de todo el aparato. Por eso esto se lee
siempre, se registre o no en el log: viaja en la telemetría, y con la placa
montada en la puerta la telemetría es la única ventana que hay.

Detalle de lectura: `0x3501/0x3502` son los 16 bits bajos del campo de 20:
líneas en los 12 altos y fracción de línea en los 4 bajos. Con el techo
limitado a 12 bits de líneas, `0x3500` siempre es cero y no hace falta leerlo.

## Inicialización

### `sccb_bus_init`

> Comentario original de `main/camera.c`.

El driver esp32-camera compilado contra IDF >= 5.4 usa `sccb-ng.c`, que
recupera el bus con `i2c_master_get_bus_handle(port)`. Por eso hay que crear
el bus con la API nueva (`i2c_new_master_bus`) y no con la legacy.

### `mirilla_camera_init`

Notas del cuerpo:

- En `camera_config_t`, -1 en `pin_sccb_sda` le dice al driver que use `sccb_i2c_port` en vez de pines.
- Reintentos: los primeros accesos SCCB tras configurar el sensor a veces
  fallan, y de esto no queremos quedarnos sin saber si cuató (3 intentos con 20ms de espera).
- Si la política de exposición NO se confirmó: el sensor se queda con los
  límites por defecto del driver y la escena saldrá oscura.

### Rotación 180º (`CONFIG_MIRILLA_ROTATE_180`)

> Comentario original de `main/camera.c`.

La placa va montada boca abajo en la mirilla, así que la imagen sale
invertida. Un giro de 180 grados es vflip + hmirror, y el OV5640 los
aplica en su propio ISP antes de comprimir: son escrituras I2C una sola
vez al arrancar, sin coste por frame ni pérdida de calidad. La
alternativa (girarlo en el servidor) obligaría a descodificar y
recomprimir cada JPEG de 1080p.

### Benchmark (`mirilla_camera_benchmark`)

> Comentario original de `main/camera.c`.

Tras el benchmark el AEC ya ha convergido: es el mejor momento para ver
con qué exposición y ganancia se ha quedado.

## API pública (camera.h)

### `mirilla_camera_aec_t`

> Comentario original de `main/camera.h`.

Último estado del AEC/AGC leído del sensor por la vigilancia periódica.

`frame_ms` es el dato que importa para el ritmo: es la duración real del
frame ((VTS + líneas de relleno) x reloj de línea). A 1080p el driver DVP
entrega un frame de cada dos, así que el techo del aparato es
`1000/(2 x frame_ms)` FPS: con los 84.63ms del modo día salen 5.9. Si
`frame_ms` sube sin que nadie haya tocado la configuración, es el AEC
alargando el frame, y cada milisegundo cuesta el doble.

El resto distingue "no hay luz" de "el control está topado": con
`exposure_ms == ceiling_ms`, la ganancia en su techo y `avg` todavía por
debajo de `MIRILLA_AEC_TARGET`, el AEC ha hecho lo que ha podido.

Campos:

| Campo | Significado |
|---|---|
| `valid` | false mientras no se haya podido leer por SCCB |
| `exposure_ms` | exposición elegida por el AEC |
| `ceiling_ms` | techo que se le ha impuesto |
| `frame_ms` | duración real del frame |
| `gain_x100` | ganancia analógica, x100 (1250 = 12.5x) |
| `avg` | media medida en la ventana del AEC, 0-255 |

### Funciones

- `mirilla_camera_init`: Inicializa el bus I2C compartido (SCCB) y a continuación el sensor OV5640
  por DVP. Debe llamarse una sola vez, antes de cualquier captura.
- `mirilla_camera_resolution_str`: Descripción legible de la resolución configurada, p.ej. "1920x1080".
- `mirilla_camera_keep_exposure_policy`: Reafirma que el bloque AEC/AGC del sensor sigue con la política de
  exposición configurada (modo día o noche, techos de exposición y ganancia,
  objetivo y ventana de medición). Pensada para llamarse desde el bucle de
  captura en cada frame: internamente solo toca el bus SCCB una vez cada
  `MIRILLA_EXPOSURE_GUARD_S` segundos, y en el caso normal la comprobación son
  lecturas de registro sin escritura.
- `mirilla_camera_benchmark`: Captura en bucle durante `seconds` sin subir nada y registra el ritmo puro
  del sensor. Permite separar el límite de la cámara del límite de la red.
- `mirilla_camera_aec`: Copia el último estado del AEC muestreado. Se refresca en cada revisión de
  la política de exposición, o sea cada `MIRILLA_EXPOSURE_GUARD_S` segundos.
