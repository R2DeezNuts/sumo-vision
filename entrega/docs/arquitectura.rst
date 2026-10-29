Arquitectura del sistema
========================

El punto de entrada es ``main.py``. Este fichero carga los cuatro bloques desde
sus carpetas, lee los ``config.json`` y ejecuta el bucle de control:

1. ``0_recibir_sumocam`` recibe los frames JPEG enviados por la ESP32-CAM.
2. ``1_procesar_imagen`` detecta las lineas del carril y objetos en la escena.
3. ``2_toma_decision`` decide si seguir el carril, activar el protocolo de
   expulsion de objetos o atender una orden manual.
4. ``3_enviar_respuesta`` traduce la accion final a comandos UDP para el robot.

La interfaz visual de OpenCV muestra una vista principal y tres paneles de
diagnostico: mascara, ROI y vista bird/debug. La configuracion se mantiene en
JSON para poder ajustar umbrales, velocidades y tiempos sin cambiar codigo.

