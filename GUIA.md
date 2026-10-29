# Guia rapida del proyecto

Este proyecto se lanza desde el portatil y coordina la recepcion de video de la ESP32-CAM, el procesado de imagen, la toma de decision y el envio de ordenes al robot SUMO.

## Lanzar el programa

Desde la carpeta del proyecto, instala las dependencias una vez:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

1. Enciende la ESP32-CAM y el robot.
2. Comprueba que el portatil, la camara y el robot estan en la misma red.
3. Abre una terminal y ejecuta:

```bash
source .venv/bin/activate
python3 main.py
```

Al arrancar, el programa escucha video UDP en `0.0.0.0:5005` y envia ordenes al robot configurado en `blocks/3_enviar_respuesta/config.json`.

## Firmware y red

`SUMO/` contiene el firmware UDP del robot y `SUMO_CAM/` el de la camara.
Cada proyecto usa su propio `.env`; para configurarlo desde cero, copia su
`.env.example` a `.env` y ajusta los valores a tu red.

La IP del robot debe coincidir con `host` en
`blocks/3_enviar_respuesta/config.json`. En la camara, `LAPTOP_IP` debe apuntar
al ordenador que ejecuta Python y `LAPTOP_PORT` debe coincidir con el puerto
del receptor, por defecto `5005`.

Con PlatformIO instalado, compila desde la raiz:

```bash
python -m pip install -r requirements-dev.txt
cp -n SUMO/.env.example SUMO/.env
cp -n SUMO_CAM/.env.example SUMO_CAM/.env
# Edita ambos .env antes de compilar. -n conserva los archivos existentes.
pio run -d SUMO
pio run -d SUMO_CAM
```

Si ya tienes los `.env` configurados, omite los dos comandos `cp`.
Las credenciales reales se guardan solo en esos archivos, excluidos de Git.
Para cargar el firmware, conecta cada placa por USB y ejecuta:

```bash
pio run -d SUMO -t upload
pio run -d SUMO_CAM -t upload
```

PlatformIO detecta el puerto serie; si tienes varios conectados, indica el
correspondiente con `--upload-port /dev/ttyUSB0`. Para ver el arranque de cada
placa, usa `pio device monitor -d SUMO` o `pio device monitor -d SUMO_CAM`.

Los valores de motores y vision son los del prototipo original. Ajustalos a
tu montaje antes de usar el modo automatico. El protocolo UDP no autentica
las ordenes: usa una red local de confianza. En automatico, perder el video
genera una parada cuando el receptor deja de entregar imagenes; el receptor
conserva brevemente la ultima imagen para tolerar perdidas puntuales. Con
imagen pero sin lineas detectadas, la estrategia original puede ordenar avanzar.
El firmware de motores no tiene temporizador de parada por perdida de conexion;
mantiene la ultima orden si deja de recibir comandos del ordenador.

## Teclas principales

- `Enter` o `espacio`: activar o bloquear el movimiento.
- `m`: cambiar entre modo automatico y modo manual.
- `w`: avanzar en modo manual.
- `a`: girar/curvar a la izquierda en modo manual.
- `d`: girar/curvar a la derecha en modo manual.
- `s`: parar en modo manual.
- `f`, `l`, `r`, `s`: forzar avance, izquierda, derecha o parada desde modo automatico.
- `q` o `Esc`: salir del programa.

Al salir, el programa envia parada al robot.

## Guia de carpetas

- `main.py`: punto de entrada. Carga los bloques, abre la ventana de OpenCV y coordina todo el flujo.
- `blocks/0_recibir_sumocam/`: recibe y reconstruye los frames UDP enviados por la ESP32-CAM.
- `blocks/1_procesar_imagen/`: procesa la imagen, detecta lineas del carril y objetos.
- `blocks/2_toma_decision/`: decide la accion final: seguir carril, protocolo de objeto, manual, activar/parar.
- `blocks/3_enviar_respuesta/`: traduce acciones a comandos UDP para el robot.
- `debug_records/`: grabaciones de depuracion generadas si `debug_record_enabled` esta activo.
- `entrega/`: documentacion final, memoria, presentacion y PDFs generados.
- `SUMO/` y `SUMO_CAM/`: firmware y configuracion de las placas.

## Herramientas de diagnostico

Con el programa principal cerrado, puedes probar el video con
`python SUMO_CAM/tools/udp_video_receiver.py`. Ambos receptores usan el mismo
puerto; ejecuta uno cada vez. El protocolo se describe en
`SUMO_CAM/SUMO_CAM.md`.

Para enviar ordenes manuales desde terminal:

```bash
python SUMO/tools/udp_control.py 192.168.1.148
python SUMO/tools/calibrate_motors.py --help
```

## Verificacion local

```bash
python -m unittest discover -s tests -v
python -m pip_audit -r requirements.txt
sphinx-build -W -b html entrega/docs entrega/docs/_build/html
```

Cada bloque tiene un `config.json` con sus parametros y un archivo `.py` con su codigo principal.
