# Guia rapida del proyecto

Este proyecto se lanza desde el portatil y coordina la recepcion de video de la ESP32-CAM, el procesado de imagen, la toma de decision y el envio de ordenes al robot SUMO.

## Lanzar el programa

1. Enciende la ESP32-CAM y el robot.
2. Comprueba que el portatil, la camara y el robot estan en la misma red.
3. Abre una terminal y ejecuta:

```bash
cd 1_TFM
python3 main.py
```

Al arrancar, el programa escucha video UDP en `0.0.0.0:5005` y envia ordenes al robot configurado en `blocks/3_enviar_respuesta/config.json`.

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

Cada bloque tiene un `config.json` con sus parametros y un archivo `.py` con su codigo principal.
