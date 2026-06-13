#!/usr/bin/env python3
"""Punto de entrada del sistema de vision y control del sumobot.

El modulo carga dinamicamente los cuatro bloques del proyecto, lee sus
ficheros de configuracion, recibe frames UDP de la ESP32-CAM, ejecuta el
procesado de imagen, aplica la toma de decision y envia ordenes UDP al robot.
Tambien construye la ventana de diagnostico de OpenCV y gestiona el modo
automatico/manual.
"""

import csv
import importlib.util
import inspect
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
BLOCKS_DIR = PROJECT_DIR / "blocks"
WINDOW_NAME = "SUMO Vision"
MODE_AUTO = "auto"
MODE_MANUAL = "manual"

ACTION_FORWARD = "forward"
ACTION_CURVE_LEFT = "curve_left"
ACTION_CURVE_RIGHT = "curve_right"
ACTION_STOP = "stop"


def movement_label(enabled):
    """Devuelve el texto corto usado para estado de movimiento."""
    return "ACTIVADO" if enabled else "BLOQUEADO"


def mode_label(mode):
    """Devuelve el nombre visible del modo de control actual."""
    return "MANUAL" if mode == MODE_MANUAL else "AUTO"


def load_symbol(module_path, symbol_name):
    """Carga un simbolo Python desde un fichero sin exigir que sea paquete.

    Args:
        module_path: Ruta al fichero Python que contiene el simbolo.
        symbol_name: Nombre de la clase, funcion o constante que se quiere
            recuperar.

    Returns:
        El objeto Python solicitado.
    """
    module_path = Path(module_path)
    module_name = f"dynamic_{module_path.stem}_{abs(hash(str(module_path)))}"
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"No se puede cargar módulo desde {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    try:
        return getattr(module, symbol_name)
    except AttributeError as exc:
        raise RuntimeError(f"No existe símbolo {symbol_name} en {module_path}") from exc


def load_json(path):
    """Lee un fichero JSON de configuracion y valida que sea un objeto."""
    if not path.exists():
        raise RuntimeError(f"No existe {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"{path} debe contener un objeto JSON")
    return data


def kwargs_for_ctor(ctor, cfg):
    """Filtra un diccionario para quedarse solo con parametros de un constructor."""
    sig = inspect.signature(ctor)
    allowed = {name for name in sig.parameters if name != "self"}
    return {k: v for k, v in cfg.items() if k in allowed}


class DebugRecorder:
    """Grabador opcional de flujos de depuracion.

    Cuando esta activado, crea una carpeta por ejecucion y guarda videos MP4
    para la vista principal, la mascara, la ROI y la vista bird/debug.
    """

    def __init__(self, enabled=False, output_dir=None, fps=20):
        """Inicializa la sesion de grabacion si `enabled` esta activo."""
        self.enabled = bool(enabled)
        self.fps = float(max(1.0, fps))
        self.output_dir = Path(output_dir) if output_dir else (PROJECT_DIR / "debug_records")
        self.session_dir = None
        self.writers = {}
        if self.enabled:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_dir = self.output_dir / f"run_{stamp}"
            self.session_dir.mkdir(parents=True, exist_ok=True)
            print(f"[debug_record] grabando en {self.session_dir}")

    def _open_writer(self, stream_name, frame):
        if not self.enabled:
            return None
        h, w = frame.shape[:2]
        path = self.session_dir / f"{stream_name}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(path), fourcc, self.fps, (w, h))
        if not writer.isOpened():
            raise RuntimeError(f"No se pudo crear video de debug: {path}")
        self.writers[stream_name] = writer
        return writer

    def write(self, stream_name, frame):
        """Escribe un frame en el video asociado a `stream_name`."""
        if not self.enabled or frame is None:
            return
        if frame.ndim == 2:
            out = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            out = frame
        writer = self.writers.get(stream_name)
        if writer is None:
            writer = self._open_writer(stream_name, out)
        writer.write(out)

    def close(self):
        """Cierra todos los escritores de video abiertos."""
        for writer in self.writers.values():
            writer.release()
        self.writers.clear()


class ManualLabelRecorder:
    """Registra etiquetas y decisiones por frame para analisis posterior."""

    def __init__(self, debug_recorder):
        """Asocia el registro manual a la misma sesion del grabador de debug."""
        self.debug_recorder = debug_recorder
        self.enabled = bool(debug_recorder.enabled)
        self.file = None
        self.writer = None
        self.path = None

    def _ensure_open(self):
        if not self.enabled:
            return
        if self.writer is not None:
            return
        session_dir = self.debug_recorder.session_dir
        if session_dir is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_dir = Path("debug_records") / f"manual_{stamp}"
            session_dir.mkdir(parents=True, exist_ok=True)
        self.path = session_dir / "manual_labels.csv"
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=[
                "frame_idx",
                "time_s",
                "mode",
                "key",
                "manual_action",
                "manual_delta",
                "sent_action",
                "sent_delta",
                "vision_reason",
                "vision_confidence",
                "vision_offset_px",
                "vision_action",
                "vision_steering_percent",
            ],
        )
        self.writer.writeheader()
        print(f"[manual_record] etiquetas en {self.path}")

    def write(self, frame_idx, time_s, mode, key, manual_action, manual_delta, sent_action, sent_delta, proc_result):
        """Anade una fila CSV con estado de teclado, vision y orden enviada."""
        if not self.enabled:
            return
        self._ensure_open()
        offset = proc_result.get("offset_px")
        self.writer.writerow(
            {
                "frame_idx": frame_idx,
                "time_s": f"{time_s:.3f}",
                "mode": mode,
                "key": key,
                "manual_action": manual_action or "",
                "manual_delta": "" if manual_delta is None else manual_delta,
                "sent_action": sent_action or "",
                "sent_delta": "" if sent_delta is None else sent_delta,
                "vision_reason": proc_result.get("reason", ""),
                "vision_confidence": f"{float(proc_result.get('confidence', 0.0)):.4f}",
                "vision_offset_px": "" if offset is None else f"{float(offset):.2f}",
                "vision_action": proc_result.get("action", ""),
                "vision_steering_percent": proc_result.get("steering_percent", 0),
            }
        )

    def close(self):
        """Cierra el CSV de etiquetas si fue creado."""
        if self.file is not None:
            self.file.close()
        self.file = None
        self.writer = None


def draw_waiting_frame(width, height, movement_enabled, port):
    """Construye la pantalla mostrada mientras no llega video UDP."""
    out = np.zeros((height, width, 3), dtype=np.uint8)
    lines = [
        "Esperando video UDP de ESP32-CAM...",
        f"Puerto UDP: {port}",
        f"Movimiento: {movement_label(movement_enabled)} | m=auto/manual | Enter activa/para | q/Esc sale",
    ]
    y = 180
    for line in lines:
        cv2.putText(out, line, (28, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, cv2.LINE_AA)
        y += 38
    return out


def draw_overlay(frame, movement_enabled, proc_result):
    """Dibuja sobre el frame principal el estado de movimiento y vision."""
    out = frame.copy()
    _, w = out.shape[:2]
    action = proc_result.get("action", "n/a")
    steer = proc_result.get("steering_percent", 0)
    conf = proc_result.get("confidence", 0.0)
    reason = proc_result.get("reason", "n/a")
    text = (
        f"Mov={movement_label(movement_enabled)} | auto={action}:{steer}% conf={conf:.2f} ({reason}) "
        "| m=auto/manual | Enter activa/para | q/Esc"
    )
    cv2.rectangle(out, (0, 0), (w, 34), (0, 0, 0), thickness=-1)
    cv2.putText(out, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 255, 0), 2, cv2.LINE_AA)
    return out


def key_name(key):
    """Convierte el codigo de tecla de OpenCV en una etiqueta legible."""
    if key in (255, -1):
        return ""
    if key in (10, 13):
        return "enter"
    if key == 27:
        return "esc"
    if 32 <= key <= 126:
        return chr(key)
    return str(key)


def manual_action_from_key(key, curve_delta):
    """Traduce teclas del modo manual a acciones de alto nivel."""
    key = key & 0xFF
    if key == ord("w"):
        return ACTION_FORWARD, None
    if key == ord("a"):
        return ACTION_CURVE_LEFT, curve_delta
    if key == ord("d"):
        return ACTION_CURVE_RIGHT, curve_delta
    if key == ord("s"):
        return ACTION_STOP, None
    return None, None


def empty_decision(movement_enabled):
    """Crea una decision neutra con la forma esperada por el bucle principal."""
    return {
        "exit": False,
        "toggled": False,
        "movement_enabled": movement_enabled,
        "action": None,
        "delta": None,
    }


def handle_manual_key(
    key,
    movement_enabled,
    manual_action,
    manual_delta,
    curve_delta,
    allow_motion_keys=True,
    hold_action=True,
    stop_when_blocked=False,
):
    """Actualiza el estado manual a partir de una tecla de OpenCV."""
    key = key & 0xFF
    decision = empty_decision(movement_enabled)

    if key in (10, 13, ord(" ")):
        movement_enabled = not movement_enabled
        decision.update(toggled=True, movement_enabled=movement_enabled)
        if not movement_enabled:
            manual_action, manual_delta = ACTION_STOP, None
    elif key in (27, ord("q")):
        decision["exit"] = True
    elif allow_motion_keys:
        next_action, next_delta = manual_action_from_key(key, curve_delta)
        if next_action is not None:
            manual_action, manual_delta = next_action, next_delta
    elif key == ord("s"):
        manual_action, manual_delta = ACTION_STOP, None
        decision["action"] = ACTION_STOP

    if movement_enabled and hold_action:
        decision["action"] = manual_action
        decision["delta"] = manual_delta
    elif stop_when_blocked and key == ord("s"):
        decision["action"] = ACTION_STOP
    return decision, manual_action, manual_delta


def report_toggle(decision, sender):
    """Informa de cambios de movimiento y manda parada al desactivar."""
    if not decision["toggled"]:
        return
    print("Movimiento ACTIVADO" if decision["movement_enabled"] else "Movimiento PARADO")
    if not decision["movement_enabled"]:
        sender.stop()


def send_decision(sender, decision):
    """Envia la accion de una decision y devuelve lo realmente solicitado."""
    action = decision["action"]
    delta = decision.get("delta")
    if action is not None:
        sender.send_action(action, delta=delta)
    return action, delta


def show_waiting_window(cfg_procesar, cfg_recibir, decision_maker):
    """Dibuja la pantalla de espera de video."""
    cv2.imshow(
        WINDOW_NAME,
        draw_waiting_frame(
            cfg_procesar.get("width", 640),
            480,
            decision_maker.movement_enabled,
            cfg_recibir["port"],
        ),
    )


def write_debug_frames(debug_recorder, frame_main, proc_result):
    """Guarda las cuatro vistas de debug del frame actual."""
    debug_recorder.write("main", frame_main)
    debug_recorder.write("mask", proc_result["debug_mask"])
    debug_recorder.write("roi", proc_result["debug_roi"])
    debug_recorder.write("bird", proc_result["debug_bird"])


def _as_bgr(frame):
    """Normaliza una imagen a formato BGR de tres canales para la interfaz."""
    if frame is None:
        return np.zeros((240, 320, 3), dtype=np.uint8)
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame


def _fit_panel(frame, width, height):
    """Escala un frame dentro de un panel manteniendo relacion de aspecto."""
    frame = _as_bgr(frame)
    h, w = frame.shape[:2]
    scale = min(width / max(1, w), height / max(1, h))
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    out = np.zeros((height, width, 3), dtype=np.uint8)
    x = (width - new_w) // 2
    y = (height - new_h) // 2
    out[y:y + new_h, x:x + new_w] = resized
    return out


def _draw_panel_label(frame, title):
    """Dibuja el titulo de un panel de diagnostico."""
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 30), (0, 0, 0), thickness=-1)
    cv2.putText(frame, title, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2, cv2.LINE_AA)
    return frame


def _object_protocol_text(decision):
    """Devuelve el texto visible del protocolo de expulsion de objeto."""
    if not decision or not decision.get("object_protocol_active"):
        return None
    state = str(decision.get("object_state", decision.get("box_state", "active"))).upper()
    return f"PROTOCOLO OBJETO: {state}"


def _draw_object_protocol_banner(frame, decision):
    """Superpone una banda de aviso cuando el protocolo de objeto esta activo."""
    text = _object_protocol_text(decision)
    if text is None:
        return frame
    out = frame.copy()
    cv2.rectangle(out, (0, 34), (out.shape[1], 70), (0, 110, 255), thickness=-1)
    cv2.putText(out, text, (12, 59), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(out, text, (12, 59), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def compose_dashboard(frame_main, proc_result, decision, movement_enabled, mode=MODE_AUTO, manual_action=None, manual_delta=None):
    """Compone la ventana 2x2 de diagnostico y estado de control."""
    panel_w = 640
    panel_h = 360
    header_h = 104
    out = np.zeros((header_h + 2 * panel_h, 2 * panel_w, 3), dtype=np.uint8)

    reason = proc_result.get("reason", "n/a")
    conf = float(proc_result.get("confidence", 0.0))
    offset = proc_result.get("offset_px")
    proc_action = proc_result.get("action", "n/a")
    proc_steer = proc_result.get("steering_percent", 0)
    final_action = decision.get("action")
    final_delta = decision.get("delta")
    if final_action is None:
        final_text = "none"
    elif final_delta is None:
        final_text = str(final_action)
    else:
        final_text = f"{final_action}:{final_delta}"
    offset_text = "n/a" if offset is None else f"{float(offset):+.1f}px"
    if manual_action is None:
        manual_text = "none"
    elif manual_delta is None:
        manual_text = manual_action
    else:
        manual_text = f"{manual_action}:{manual_delta}"

    line1 = (
        f"Modo={mode_label(mode)} Mov={movement_label(movement_enabled)} | Detecta={reason} "
        f"conf={conf:.2f} off={offset_text} | "
        f"Proc={proc_action}:{proc_steer}% | Manual={manual_text} | Accion={final_text}"
    )
    line2 = "m=auto/manual | Enter activa/para | MANUAL: w=F a=LL d=RR s=stop | AUTO: f/l/r/s | q/Esc sale"
    object_text = _object_protocol_text(decision)
    line3 = object_text or "PROTOCOLO OBJETO: inactivo"
    cv2.rectangle(out, (0, 0), (out.shape[1], header_h), (0, 0, 0), thickness=-1)
    cv2.putText(out, line1, (12, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(out, line2, (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2, cv2.LINE_AA)
    color3 = (0, 180, 255) if object_text else (120, 120, 120)
    cv2.putText(out, line3, (12, 87), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color3, 2, cv2.LINE_AA)

    panels = [
        ("Camara + decision", _draw_object_protocol_banner(frame_main, decision)),
        ("Mascara", proc_result.get("debug_mask")),
        ("ROI detectada", proc_result.get("debug_roi")),
        ("Bird/debug", proc_result.get("debug_bird")),
    ]
    for idx, (title, frame) in enumerate(panels):
        x = (idx % 2) * panel_w
        y = header_h + (idx // 2) * panel_h
        panel = _fit_panel(frame, panel_w, panel_h)
        panel = _draw_panel_label(panel, title)
        out[y:y + panel_h, x:x + panel_w] = panel
    cv2.line(out, (panel_w, header_h), (panel_w, out.shape[0] - 1), (60, 60, 60), 1)
    cv2.line(out, (0, header_h + panel_h), (out.shape[1] - 1, header_h + panel_h), (60, 60, 60), 1)
    return out


def main():
    """Ejecuta el bucle principal de recepcion, vision, decision y envio."""
    receiver_dir = BLOCKS_DIR / "0_recibir_sumocam"
    processor_dir = BLOCKS_DIR / "1_procesar_imagen"
    decision_dir = BLOCKS_DIR / "2_toma_decision"
    sender_dir = BLOCKS_DIR / "3_enviar_respuesta"

    UdpVideoReceiver = load_symbol(receiver_dir / "receiver.py", "UdpVideoReceiver")
    ImageProcessor = load_symbol(processor_dir / "processor.py", "ImageProcessor")
    DecisionMaker = load_symbol(decision_dir / "decision.py", "DecisionMaker")
    RobotCommandSender = load_symbol(sender_dir / "sender.py", "RobotCommandSender")

    cfg_recibir = load_json(receiver_dir / "config.json")
    cfg_procesar = load_json(processor_dir / "config.json")
    cfg_decidir = load_json(decision_dir / "config.json")
    cfg_enviar = load_json(sender_dir / "config.json")

    bloque_recibir = UdpVideoReceiver(
        cfg_recibir["host"],
        cfg_recibir["port"],
        frame_timeout=cfg_recibir.get("frame_timeout", 0.2),
        rotation=cfg_recibir.get("rotation"),
    )
    bloque_procesar = ImageProcessor(**kwargs_for_ctor(ImageProcessor.__init__, cfg_procesar))
    bloque_decidir = DecisionMaker(**kwargs_for_ctor(DecisionMaker.__init__, cfg_decidir))
    bloque_enviar = RobotCommandSender(**kwargs_for_ctor(RobotCommandSender.__init__, cfg_enviar))
    debug_recorder = DebugRecorder(
        enabled=cfg_procesar.get("debug_record_enabled", False),
        output_dir=cfg_procesar.get("debug_record_dir"),
        fps=cfg_procesar.get("debug_record_fps", 20),
    )
    manual_labels = ManualLabelRecorder(debug_recorder)

    print(f"[recibir_sumocam] UDP {cfg_recibir['host']}:{cfg_recibir['port']}")
    print(f"[enviar_respuesta] robot UDP {bloque_enviar.target[0]}:{bloque_enviar.target[1]}")

    last_report = time.monotonic()
    last_waiting_report = 0.0
    started_at = time.monotonic()
    frames = 0
    frame_idx = 0
    control_mode = MODE_AUTO
    manual_action = ACTION_STOP
    manual_delta = None
    manual_curve_delta = cfg_enviar.get("manual_curve_delta", cfg_decidir.get("manual_curve_delta", 10))

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    show_waiting_window(cfg_procesar, cfg_recibir, bloque_decidir)
    try:
        cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    except cv2.error:
        cv2.resizeWindow(WINDOW_NAME, 1280, 794)
    cv2.waitKey(1)

    try:
        key = 255
        while True:
            frame = bloque_recibir.read()
            if key == ord("m"):
                control_mode = MODE_MANUAL if control_mode == MODE_AUTO else MODE_AUTO
                manual_action = ACTION_STOP
                manual_delta = None
                bloque_enviar.stop()
                print(f"Modo {mode_label(control_mode)}")
                key = 255
            if frame is None:
                now = time.monotonic()
                if now - last_waiting_report >= 2.0:
                    print(f"[recibir_sumocam] esperando UDP en {cfg_recibir['host']}:{cfg_recibir['port']}...")
                    last_waiting_report = now
                show_waiting_window(cfg_procesar, cfg_recibir, bloque_decidir)
                key = cv2.waitKey(1) & 0xFF

                if control_mode == MODE_MANUAL:
                    decision, manual_action, manual_delta = handle_manual_key(
                        key,
                        bloque_decidir.movement_enabled,
                        manual_action,
                        manual_delta,
                        manual_curve_delta,
                        allow_motion_keys=False,
                        hold_action=False,
                        stop_when_blocked=True,
                    )
                    bloque_decidir.movement_enabled = decision["movement_enabled"]
                else:
                    decision = bloque_decidir.step(key, None)

                report_toggle(decision, bloque_enviar)
                send_decision(bloque_enviar, decision)
                if decision["exit"]:
                    break
                continue

            proc_result = bloque_procesar.process(frame)
            if control_mode == MODE_MANUAL:
                decision, manual_action, manual_delta = handle_manual_key(
                    key,
                    bloque_decidir.movement_enabled,
                    manual_action,
                    manual_delta,
                    manual_curve_delta,
                )
                bloque_decidir.movement_enabled = decision["movement_enabled"]
            else:
                decision = bloque_decidir.step(key, proc_result)

            report_toggle(decision, bloque_enviar)
            action_to_send, delta_to_send = send_decision(bloque_enviar, decision)

            frame_main = draw_overlay(proc_result["frame"], bloque_decidir.movement_enabled, proc_result)
            dashboard = compose_dashboard(
                frame_main,
                proc_result,
                decision,
                bloque_decidir.movement_enabled,
                mode=control_mode,
                manual_action=manual_action if control_mode == MODE_MANUAL else None,
                manual_delta=manual_delta if control_mode == MODE_MANUAL else None,
            )
            cv2.imshow(WINDOW_NAME, dashboard)
            write_debug_frames(debug_recorder, frame_main, proc_result)
            manual_labels.write(
                frame_idx,
                time.monotonic() - started_at,
                control_mode,
                key_name(key),
                manual_action if control_mode == MODE_MANUAL else None,
                manual_delta if control_mode == MODE_MANUAL else None,
                action_to_send,
                delta_to_send,
                proc_result,
            )
            frames += 1
            frame_idx += 1
            now = time.monotonic()
            if now - last_report >= 2.0:
                print(
                    f"FPS: {frames / (now - last_report):.1f} | "
                    f"modo={mode_label(control_mode)} | "
                    f"movimiento={'ON' if bloque_decidir.movement_enabled else 'OFF'}"
                )
                frames = 0
                last_report = now
            key = cv2.waitKey(1) & 0xFF
            if decision["exit"]:
                break
    finally:
        bloque_enviar.stop()
        bloque_enviar.close()
        manual_labels.close()
        debug_recorder.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
