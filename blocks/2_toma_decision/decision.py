"""Toma de decision para seguimiento de carril y expulsion de objetos.

La clase principal combina tres comportamientos: habilitacion manual de
movimiento, seguimiento automatico de las lineas detectadas y una rutina
reactiva para empujar un objeto fuera del carril y reincorporarse mediante
marcha atras curva.
"""

import time

import numpy as np

ACTION_FORWARD = "forward"
ACTION_CURVE_LEFT = "curve_left"
ACTION_CURVE_RIGHT = "curve_right"
ACTION_BACK_CURVE_LEFT = "back_curve_left"
ACTION_STOP = "stop"


class DecisionMaker:
    """Controlador de alto nivel que decide la accion final por frame."""

    def __init__(
        self,
        start_enabled=False,
        conf_min=0.2,
        deadband_px=15,
        min_delta=1,
        max_delta=10,
        kp=0.06,
        ki=0.001,
        kd=0.001,
        box_eject_enabled=True,
        box_trigger_min_area=10000,
        box_approach_frames=70,
        box_push_frames=70,
        box_rejoin_back_frames=70,
        box_cooldown_frames=20,
    ):
        """Inicializa los parametros principales de control automatico.

        Los ajustes secundarios de esquina, enfoque y seguridad se mantienen
        como constantes internas para que el JSON de entrega sea facil de usar.
        """
        self.movement_enabled = bool(start_enabled)
        self.conf_min = float(np.clip(conf_min, 0.0, 1.0))
        self.deadband_px = float(max(0.0, deadband_px))
        self.min_delta = int(np.clip(min_delta, 0, 100))
        self.max_delta = int(np.clip(max_delta, self.min_delta, 100))
        self.kp = float(max(0.0, kp))
        self.ki = float(max(0.0, ki))
        self.kd = float(max(0.0, kd))
        self.integral_limit = 250.0
        self.manual_curve_delta = int(np.clip(10, self.min_delta, self.max_delta))
        self.corner_delay_s = 0.25
        self.corner_delta = int(np.clip(6, self.min_delta, self.max_delta))
        self.corner_lost_hold_s = 1.2
        self.box_eject_enabled = bool(box_eject_enabled)
        self.box_trigger_min_area = float(max(100.0, box_trigger_min_area))
        self.box_trigger_min_bottom_ratio = 0.45
        self.box_trigger_max_center_offset_ratio = 0.35
        self.box_focus_max_frames = 20
        self.box_focus_delta = int(np.clip(6, self.min_delta, self.max_delta))
        self.box_focus_center_tolerance_px = 4.0
        self.box_push_frames = int(np.clip(box_push_frames, 1, 180))
        self.box_push_curve_delta = int(np.clip(8, 0, self.max_delta))
        self.box_approach_frames = int(np.clip(box_approach_frames, 0, 120))
        self.box_rejoin_back_frames = int(np.clip(box_rejoin_back_frames, 1, 240))
        self.box_rejoin_curve_delta = int(np.clip(8, 0, self.max_delta))
        self.box_cooldown_frames = int(np.clip(box_cooldown_frames, 0, 180))

        self._last_error = None
        self._integral = 0.0
        self._last_time = None
        self._single_line_reason = None
        self._single_line_since = None
        self._corner_action = None
        self._corner_seen_at = None
        self._box_state = "idle"
        self._box_state_frames = 0
        self._box_focus_elapsed = 0
        self._box_cooldown_remaining = 0

    def _reset_box_eject(self, clear_cooldown=True):
        self._box_state = "idle"
        self._box_state_frames = 0
        self._box_focus_elapsed = 0
        if clear_cooldown:
            self._box_cooldown_remaining = 0

    def _finish_box_eject(self):
        self._reset_box_eject(clear_cooldown=False)
        self._box_cooldown_remaining = self.box_cooldown_frames

    def _with_status(self, result):
        result["object_state"] = self._box_state
        result["box_state"] = self._box_state
        result["object_protocol_active"] = self._box_state != "idle"
        result["box_cooldown_remaining"] = self._box_cooldown_remaining
        return result

    def _start_box_eject(self):
        self._box_state = "focus"
        self._box_state_frames = 0
        self._box_focus_elapsed = 0

    def _best_box(self, proc_result):
        if proc_result is None:
            return None
        objects = proc_result.get("objects")
        if objects is None:
            objects = proc_result.get("boxes", [])
        if not objects:
            return None
        try:
            return max(objects, key=lambda obj: float(obj.get("area", 0.0)))
        except Exception:
            return None

    def _box_trigger_ready(self, proc_result):
        if proc_result is None:
            return False
        box = self._best_box(proc_result)
        if box is None:
            return False
        area = float(box.get("area", 0.0))
        if area < self.box_trigger_min_area:
            return False
        image_w = int(proc_result.get("image_width", 0))
        if image_w > 0:
            box_center_x = float(box.get("x", 0) + 0.5 * box.get("w", 0))
            image_center_x = 0.5 * float(image_w)
            max_offset = self.box_trigger_max_center_offset_ratio * float(image_w)
            if abs(box_center_x - image_center_x) > max_offset:
                return False
        image_h = int(proc_result.get("image_height", 0))
        if image_h <= 0:
            return False
        bottom_ratio = float(box.get("y", 0) + box.get("h", 0)) / float(max(1, image_h))
        if bottom_ratio < self.box_trigger_min_bottom_ratio:
            return False
        return True

    def _step_box_eject(self, proc_result):
        if self._box_state == "idle":
            return None

        if self._box_state == "focus":
            self._box_focus_elapsed += 1
            box = self._best_box(proc_result)
            image_w = int(proc_result.get("image_width", 0)) if proc_result is not None else 0
            if box is None or image_w <= 0:
                if self._box_focus_elapsed >= self.box_focus_max_frames:
                    self._box_state = "approach"
                    self._box_state_frames = self.box_approach_frames
                return ACTION_FORWARD, None

            box_center_x = float(box.get("x", 0) + 0.5 * box.get("w", 0))
            image_center_x = 0.5 * float(image_w)
            err = box_center_x - image_center_x
            if abs(err) <= self.box_focus_center_tolerance_px:
                self._box_state = "approach"
                self._box_state_frames = self.box_approach_frames
                return ACTION_FORWARD, None

            if err > 0:
                return ACTION_CURVE_RIGHT, self.box_focus_delta
            return ACTION_CURVE_LEFT, self.box_focus_delta

        if self._box_state == "approach":
            if self._box_state_frames > 0:
                self._box_state_frames -= 1
                return ACTION_FORWARD, None
            self._box_state = "eject"
            self._box_state_frames = self.box_push_frames

        if self._box_state == "eject":
            if self._box_state_frames > 0:
                self._box_state_frames -= 1
                return ACTION_CURVE_LEFT, self.box_push_curve_delta
            self._box_state = "rejoin"
            self._box_state_frames = self.box_rejoin_back_frames

        if self._box_state == "rejoin":
            if self._box_state_frames <= 0:
                self._finish_box_eject()
                return None
            self._box_state_frames -= 1
            return ACTION_BACK_CURVE_LEFT, self.box_rejoin_curve_delta

        self._reset_box_eject()
        return None

    def _reset_pid(self):
        self._last_error = None
        self._integral = 0.0
        self._last_time = None

    def _reset_corner(self):
        self._single_line_reason = None
        self._single_line_since = None
        self._corner_action = None
        self._corner_seen_at = None

    def _single_line_action(self, reason):
        now = time.monotonic()
        if self._single_line_reason != reason:
            self._single_line_reason = reason
            self._single_line_since = now

        if self._single_line_since is None:
            self._single_line_since = now

        if now - self._single_line_since < self.corner_delay_s:
            return ACTION_FORWARD, None

        if reason.startswith("one_line_right"):
            self._corner_action = ACTION_CURVE_LEFT
        else:
            self._corner_action = ACTION_CURVE_RIGHT
        self._corner_seen_at = now
        return self._corner_action, self.corner_delta

    def _pid_delta(self, error_px):
        now = time.monotonic()
        if self._last_time is None:
            dt = 0.0
        else:
            dt = max(1e-3, now - self._last_time)

        if dt > 0.0:
            self._integral += error_px * dt
            self._integral = float(np.clip(self._integral, -self.integral_limit, self.integral_limit))

        if self._last_error is None or dt <= 0.0:
            derivative = 0.0
        else:
            derivative = (error_px - self._last_error) / dt

        self._last_error = error_px
        self._last_time = now

        output = self.kp * error_px + self.ki * self._integral + self.kd * derivative
        return float(np.clip(output, -self.max_delta, self.max_delta))

    def _line_follow_action(self, proc_result):
        if proc_result is None:
            self._reset_pid()
            self._reset_corner()
            return ACTION_FORWARD, None

        reason = str(proc_result.get("reason", ""))
        confidence = float(proc_result.get("confidence", 0.0))
        offset_px = proc_result.get("offset_px")

        if reason == "no_lines":
            self._reset_pid()
            if (
                self._corner_action is not None
                and self._corner_seen_at is not None
                and time.monotonic() - self._corner_seen_at <= self.corner_lost_hold_s
            ):
                return self._corner_action, self.corner_delta
            self._reset_corner()
            return ACTION_FORWARD, None

        if offset_px is None or confidence < self.conf_min:
            self._reset_pid()
            self._reset_corner()
            return ACTION_FORWARD, None

        if reason.startswith("one_line_"):
            self._reset_pid()
            if not bool(proc_result.get("single_line_lower_half", False)):
                self._reset_corner()
                return ACTION_FORWARD, None
            return self._single_line_action(reason)

        if not reason.startswith("two_lines"):
            self._reset_pid()
            self._reset_corner()
            return ACTION_FORWARD, None

        self._reset_corner()

        error_px = float(offset_px)
        if abs(error_px) <= self.deadband_px:
            self._reset_pid()
            return ACTION_FORWARD, None

        output = self._pid_delta(error_px)
        abs_output = abs(output)
        if abs_output <= 0.0:
            return ACTION_FORWARD, None

        delta = int(np.clip(round(abs_output), self.min_delta, self.max_delta))
        if output < 0:
            return ACTION_CURVE_LEFT, delta
        return ACTION_CURVE_RIGHT, delta

    def _auto_action(self, proc_result):
        if self.box_eject_enabled:
            if self._box_state == "idle" and self._box_cooldown_remaining > 0:
                self._box_cooldown_remaining -= 1
            if (
                self._box_state == "idle"
                and self._box_cooldown_remaining <= 0
                and self._box_trigger_ready(proc_result)
            ):
                self._start_box_eject()

            state_before = self._box_state
            box_action = self._step_box_eject(proc_result)
            if box_action is not None:
                if state_before != "rejoin":
                    self._reset_pid()
                    self._reset_corner()
                return box_action

        return self._line_follow_action(proc_result)

    def step(self, key, proc_result=None):
        """Procesa una tecla y el resultado de vision para producir una accion.

        Args:
            key: Codigo de tecla devuelto por OpenCV.
            proc_result: Diccionario generado por ``ImageProcessor.process``.

        Returns:
            Diccionario con accion, delta, estado de movimiento y estado del
            protocolo de objeto.
        """
        result = {
            "exit": False,
            "toggled": False,
            "movement_enabled": self.movement_enabled,
            "action": None,
            "delta": None,
            "object_state": self._box_state,
            "box_state": self._box_state,
            "object_protocol_active": self._box_state != "idle",
            "box_cooldown_remaining": self._box_cooldown_remaining,
        }

        if key in (10, 13, ord(" ")):
            self.movement_enabled = not self.movement_enabled
            result["toggled"] = True
            result["movement_enabled"] = self.movement_enabled
            self._reset_pid()
            self._reset_corner()
            self._reset_box_eject()
            if not self.movement_enabled:
                result["action"] = ACTION_STOP
            return self._with_status(result)

        if key == 27 or key == ord("q"):
            result["exit"] = True
            return self._with_status(result)

        if key == ord("s"):
            self._reset_pid()
            self._reset_corner()
            self._reset_box_eject()
            result["action"] = ACTION_STOP
            return self._with_status(result)

        if self.movement_enabled:
            if key == ord("f"):
                self._reset_pid()
                self._reset_corner()
                self._reset_box_eject()
                result["action"] = ACTION_FORWARD
                return self._with_status(result)
            if key == ord("l"):
                self._reset_pid()
                self._reset_corner()
                self._reset_box_eject()
                result["action"] = ACTION_CURVE_LEFT
                result["delta"] = self.manual_curve_delta
                return self._with_status(result)
            if key == ord("r"):
                self._reset_pid()
                self._reset_corner()
                self._reset_box_eject()
                result["action"] = ACTION_CURVE_RIGHT
                result["delta"] = self.manual_curve_delta
                return self._with_status(result)

            auto_action, auto_delta = self._auto_action(proc_result)
            result["action"] = auto_action
            result["delta"] = auto_delta

        return self._with_status(result)
