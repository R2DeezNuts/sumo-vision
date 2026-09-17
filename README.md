# Sumo Vision

Reutilicé un robot de sumo para un prototipo quitanieves que localiza y retira obstáculos dentro de un área delimitada. Integré las imágenes de una ESP32-CAM con OpenCV y la lógica que envía las órdenes de movimiento al robot mediante UDP.

El prototipo retiró obstáculos de forma autónoma. La [demostración](https://r2deeznuts.github.io/img/sumo-vision-preview.mp4) muestra su funcionamiento sobre la plataforma física.

## Código y documentación

El programa se inicia en [main.py](main.py). Los bloques de [blocks/](blocks/) separan la recepción de imágenes, su procesamiento, las decisiones y el envío de órdenes.

- [Guía de uso y controles](GUIA.md).
- [Memoria del proyecto](Docs/memoria_final.pdf).
- [Documentación del código](Docs/documentacion_codigo.pdf).

[Volver al portfolio](https://r2deeznuts.github.io/)
