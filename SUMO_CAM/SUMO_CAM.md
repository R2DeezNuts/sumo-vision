# ESP32-CAM: vídeo UDP

Firmware para una ESP32-CAM AI Thinker. Captura imágenes JPEG y las envía
fragmentadas al ordenador, donde se procesan con OpenCV.

## Configuración y compilación

Desde la raíz del repositorio, con el entorno Python activado:

```bash
python -m pip install -r requirements-dev.txt
cp -n SUMO_CAM/.env.example SUMO_CAM/.env
# Edita SUMO_CAM/.env con los datos de tu red.
pio run -d SUMO_CAM
pio run -d SUMO_CAM -t upload
pio device monitor -d SUMO_CAM
```

`WIFI_SSID` y `WIFI_PASSWORD` identifican la red. `LAPTOP_IP` es la IP del
ordenador que ejecuta el receptor; `LAPTOP_PORT` debe coincidir con el puerto
configurado en `blocks/0_recibir_sumocam/config.json`, por defecto `5005`.
`STATIC_IP`, `GATEWAY_IP`, `SUBNET_MASK` y `DNS_IP` configuran la red de la placa.

Los valores iniciales de `.env.example` son QVGA, calidad JPEG 12, objetivo de
20 FPS y fragmentos de hasta 1400 bytes. Son parámetros configurables, no una
garantía de rendimiento. El modelo de cámara se selecciona en
`src/board_config.h`. El firmware usa PSRAM si está disponible.

`scripts/load_env.py` valida el archivo `.env` e inyecta las macros al compilar.
Las credenciales no se guardan en el código ni se suben a Git.

## Protocolo de vídeo

Cada datagrama empieza por esta cabecera de 20 bytes, en little-endian:

| Campo | Tipo | Valor o significado |
| --- | --- | --- |
| magic | uint16 | `0xCAFE` |
| version | uint8 | `1` |
| flags | uint8 | `0` |
| frame_id | uint32 | Identificador de imagen |
| packet_id | uint16 | Índice del fragmento, desde cero |
| packet_count | uint16 | Número de fragmentos |
| frame_size | uint32 | Tamaño total del JPEG |
| payload_size | uint16 | Bytes de JPEG en este datagrama |
| reserved | uint16 | `0` |

Después va el fragmento JPEG. El receptor principal está en
`blocks/0_recibir_sumocam/receiver.py` y usa el formato Python
`<HBBIHHIHH`. Reordena los fragmentos y descarta imágenes incompletas caducadas.

## Diagnóstico y control

Para ver solo el vídeo, con la aplicación principal cerrada:

```bash
python SUMO_CAM/tools/udp_video_receiver.py
```

Pulsa `Esc` para salir. Este receptor y `main.py` no deben ejecutarse al mismo
tiempo porque escuchan en el mismo puerto.

La cámara también escucha órdenes de calidad en `CONTROL_PORT`, por defecto
`5006`. Acepta `QUALITY 18`, `JPEG_QUALITY 18` o `Q 18`; el rango es de 4 a 63.
Responde por UDP con `OK QUALITY <valor>` o un error. El ajuste se mantiene en
memoria hasta reiniciar; el valor de arranque viene de `.env`.
