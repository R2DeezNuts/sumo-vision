"""Envio de acciones de alto nivel al robot SUMO por UDP.

El modulo traduce acciones semanticas del controlador a comandos ASCII que
entiende el firmware de motores. Tambien limita la frecuencia de envio y aplica
una parada segura al cerrar el programa.
"""

import socket
import time

ACTION_FORWARD = "forward"
ACTION_BACKWARD = "backward"
ACTION_CURVE_LEFT = "curve_left"
ACTION_CURVE_RIGHT = "curve_right"
ACTION_BACK_CURVE_LEFT = "back_curve_left"
ACTION_BACK_CURVE_RIGHT = "back_curve_right"
ACTION_LEFT = "left"
ACTION_RIGHT = "right"
ACTION_STOP = "stop"

DIRECT_ACTIONS = {
    ACTION_FORWARD: ("F", "speed"),
    ACTION_BACKWARD: ("BK", "speed"),
    ACTION_LEFT: ("L", "turn_speed"),
    ACTION_RIGHT: ("R", "turn_speed"),
}

CURVE_ACTIONS = {
    ACTION_CURVE_LEFT: ("LL", False),
    ACTION_CURVE_RIGHT: ("RR", False),
    ACTION_BACK_CURVE_LEFT: ("BL", True),
    ACTION_BACK_CURVE_RIGHT: ("BR", True),
}


def clip_percent(value):
    """Limita una velocidad o correccion al rango aceptado por el firmware."""
    return max(0, min(100, int(value)))


class RobotCommandSender:
    """Cliente UDP para enviar ordenes de movimiento al robot."""

    def __init__(self, host, port, speed=25, turn_speed=30, interval=0.12):
        """Configura destino, velocidades nominales y periodo minimo de envio."""
        self.target = (host, int(port))
        self.speed = clip_percent(speed)
        self.turn_speed = clip_percent(turn_speed)
        self.interval = max(0.02, float(interval))
        self.last_command = None
        self.last_sent_at = 0.0
        self.back_curve_supported = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.0)

    def _drain_responses(self):
        previous_timeout = self.sock.gettimeout()
        self.sock.settimeout(0.0)
        try:
            while True:
                try:
                    self.sock.recvfrom(256)
                except (BlockingIOError, socket.timeout):
                    break
        finally:
            self.sock.settimeout(previous_timeout)

    def send_command(self, command, force=False, response_timeout=None):
        """Envia un comando ASCII crudo respetando antirrebote temporal.

        Args:
            command: Texto exacto que recibira el firmware.
            force: Si es ``True``, ignora repeticion y periodo minimo.
            response_timeout: Tiempo opcional para esperar una respuesta UDP.

        Returns:
            Respuesta textual del robot o ``None`` si no llega ninguna.
        """
        now = time.monotonic()
        changed = command != self.last_command
        if not force and (not changed or now - self.last_sent_at < self.interval):
            return None

        self.last_command = command
        self.last_sent_at = now

        if force or changed:
            self._drain_responses()
        self.sock.sendto(command.encode("ascii"), self.target)
        previous_timeout = self.sock.gettimeout()
        if response_timeout is not None:
            self.sock.settimeout(float(response_timeout))
        try:
            data, _ = self.sock.recvfrom(256)
            response = data.decode("utf-8", errors="replace").strip()
            return response
        except (BlockingIOError, socket.timeout):
            return None
        finally:
            if response_timeout is not None:
                self.sock.settimeout(previous_timeout)

    def _send_curve(self, action, delta):
        code, reverse = CURVE_ACTIONS[action]
        use_delta = self.turn_speed if delta is None else clip_percent(delta)

        if not reverse:
            self.send_command(f"{code} {self.speed} {use_delta}", force=False)
            return

        if not self.back_curve_supported:
            self.send_command(f"BK {self.speed}", force=False)
            return

        response = self.send_command(
            f"{code} {self.speed} {use_delta}",
            force=False,
            response_timeout=0.25,
        )
        if response is not None and response.startswith("ERROR"):
            self.back_curve_supported = False
            self.send_command(f"BK {self.speed}", force=True)

    def send_action(self, action, delta=None):
        """Convierte una accion de alto nivel en el comando UDP correspondiente."""
        if action in DIRECT_ACTIONS:
            code, attr = DIRECT_ACTIONS[action]
            self.send_command(f"{code} {getattr(self, attr)}", force=False)
        elif action in CURVE_ACTIONS:
            self._send_curve(action, delta)
        elif action == ACTION_STOP:
            self.send_command("S", force=True)

    def stop(self):
        """Envia varias ordenes de parada para dejar el robot en estado seguro."""
        for _ in range(3):
            self.sock.sendto(b"S", self.target)
            time.sleep(0.03)
        self.last_command = "S"
        self.last_sent_at = time.monotonic()

    def close(self):
        """Cierra el socket UDP."""
        self.sock.close()
