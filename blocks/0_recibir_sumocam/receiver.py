"""Recepcion de video UDP enviado por la ESP32-CAM.

El modulo reconstruye frames JPEG fragmentados en datagramas UDP mediante una
cabecera ligera con identificador de frame, numero de paquete y tamano total.
La clase mantiene un ultimo frame valido durante una pequena ventana de gracia
para evitar parones de interfaz ante perdidas puntuales de paquetes.
"""

import socket
import struct
import time

import cv2
import numpy as np


HEADER_FORMAT = "<HBBIHHIHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MAGIC = 0xCAFE
VERSION = 1


class UdpVideoReceiver:
    """Reconstruye imagenes OpenCV a partir de paquetes UDP fragmentados."""

    _ROTATION_CODES = {
        "0": None,
        "180": cv2.ROTATE_180,
    }

    def __init__(self, host, port, frame_timeout=0.2, rotation=None):
        """Abre el socket UDP de recepcion.

        Args:
            host: Interfaz local en la que escuchar, por ejemplo ``0.0.0.0``.
            port: Puerto UDP donde envia la ESP32-CAM.
            frame_timeout: Tiempo maximo para completar un frame fragmentado.
            rotation: Rotacion opcional de la imagen decodificada.
        """
        self.rotation_code = self._parse_rotation(rotation)
        self.frame_timeout = float(frame_timeout)
        self.frame_grace = float(max(0.12, self.frame_timeout * 1.8))
        self.frames = {}
        self.last_image = None
        self.last_image_at = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((host, int(port)))
        # Avoid long UI stalls while tolerating short packet gaps.
        recv_timeout = self.frame_timeout / 3.0 if self.frame_timeout > 0 else 0.03
        self.sock.settimeout(float(np.clip(recv_timeout, 0.01, 0.12)))

    @classmethod
    def _parse_rotation(cls, rotation):
        if rotation is None:
            return None
        key = str(rotation).strip().lower().replace("-", "_")
        if key not in cls._ROTATION_CODES:
            valid = "0, none, 180"
            raise ValueError(f"Rotacion no soportada: {rotation!r}. Valores validos: {valid}")
        return cls._ROTATION_CODES[key]

    def read(self):
        """Devuelve el siguiente frame BGR completo o ``None`` si no hay video."""
        while True:
            try:
                packet, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                now = time.monotonic()
                if self.last_image is not None and now - self.last_image_at <= self.frame_grace:
                    return self.last_image
                return None
            now = time.monotonic()

            for frame_id, state in list(self.frames.items()):
                if now - state["updated_at"] > self.frame_timeout:
                    del self.frames[frame_id]

            if len(packet) < HEADER_SIZE:
                continue

            payload = packet[HEADER_SIZE:]
            (
                magic,
                version,
                _flags,
                frame_id,
                packet_id,
                packet_count,
                frame_size,
                payload_size,
                _reserved,
            ) = struct.unpack(HEADER_FORMAT, packet[:HEADER_SIZE])

            if magic != MAGIC or version != VERSION:
                continue
            if packet_id >= packet_count or payload_size != len(payload):
                continue
            if packet_count == 0 or frame_size == 0:
                continue

            state = self.frames.get(frame_id)
            if state is None:
                state = {
                    "parts": [None] * packet_count,
                    "received": 0,
                    "frame_size": frame_size,
                    "updated_at": now,
                }
                self.frames[frame_id] = state

            if len(state["parts"]) != packet_count or state["frame_size"] != frame_size:
                del self.frames[frame_id]
                continue

            if state["parts"][packet_id] is None:
                state["parts"][packet_id] = payload
                state["received"] += 1
                state["updated_at"] = now

            if state["received"] != packet_count:
                continue

            jpeg = b"".join(state["parts"])
            del self.frames[frame_id]
            if len(jpeg) != frame_size:
                continue

            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                if self.rotation_code is not None:
                    image = cv2.rotate(image, self.rotation_code)
                self.last_image = image
                self.last_image_at = now
                return image
