"""Pruebas de integración sin hardware, red ni ventanas gráficas."""

import socket
import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import main as app


def load_block(folder, filename, symbol):
    directory = ROOT / "blocks" / folder
    cls = app.load_symbol(directory / filename, symbol)
    config = app.load_json(directory / "config.json")
    return cls, config


class PipelineTests(unittest.TestCase):
    def test_lane_processing_and_dashboard(self):
        cls, config = load_block("1_procesar_imagen", "processor.py", "ImageProcessor")
        processor = cls(**app.kwargs_for_ctor(cls.__init__, config))
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        for x in (170, 470):
            cv2.line(image, (x, 0), (x, 479), (0, 0, 0), 16)
        result = processor.process(image)
        self.assertTrue(result["reason"].startswith("two_lines"))
        self.assertLess(abs(result["offset_px"]), 5)
        cls, config = load_block("2_toma_decision", "decision.py", "DecisionMaker")
        controller = cls(**app.kwargs_for_ctor(cls.__init__, config))
        self.assertFalse(controller.movement_enabled)
        controller.step(ord(" "), result)
        decision = controller.step(255, result)
        self.assertEqual(decision["action"], "forward")
        dashboard = app.compose_dashboard(result["frame"], result, decision, True)
        self.assertEqual(dashboard.ndim, 3)
        self.assertEqual(dashboard.shape[2], 3)

    def test_movement_starts_blocked_and_can_be_stopped(self):
        cls, config = load_block("2_toma_decision", "decision.py", "DecisionMaker")
        controller = cls(**app.kwargs_for_ctor(cls.__init__, config))
        self.assertFalse(controller.step(255, None)["movement_enabled"])
        self.assertIsNone(controller.step(255, None)["action"])
        self.assertTrue(controller.step(ord(" "), None)["movement_enabled"])
        self.assertEqual(controller.step(ord(" "), None)["action"], "stop")
        self.assertTrue(controller.step(ord("q"), None)["exit"])

    def test_udp_jpeg_reassembly_with_out_of_order_packets(self):
        cls, config = load_block("0_recibir_sumocam", "receiver.py", "UdpVideoReceiver")
        image = np.full((48, 64, 3), (25, 80, 160), dtype=np.uint8)
        image[:24] = (180, 20, 40)
        ok, encoded = cv2.imencode(".jpg", image)
        self.assertTrue(ok)
        jpeg = encoded.tobytes()
        parts = [jpeg[i:i + 128] for i in range(0, len(jpeg), 128)]
        # Protocolo documentado de la cámara: cabecera LE de 20 bytes.
        packets = [
            struct.pack("<HBBIHHIHH", 0xCAFE, 1, 0, 42, index, len(parts),
                        len(jpeg), len(part), 0) + part
            for index, part in enumerate(parts)
        ]
        sock = Mock()
        sock.recvfrom.side_effect = [(b"invalido", ("127.0.0.1", 5005))] + [
            (packet, ("127.0.0.1", 5005)) for packet in reversed(packets)
        ]
        with patch("socket.socket", return_value=sock):
            receiver = cls(**config)
            result = receiver.read()
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(result, cv2.rotate(decoded, cv2.ROTATE_180))

    def test_lost_video_stops_even_during_object_ejection(self):
        cls, _ = load_block("2_toma_decision", "decision.py", "DecisionMaker")
        for state in ("idle", "focus", "approach", "eject", "rejoin"):
            with self.subTest(state=state):
                controller = cls(start_enabled=True)
                controller._box_state = state
                controller._box_state_frames = 70
                decision = controller.step(255, None)
                self.assertEqual(decision["action"], "stop")
                self.assertFalse(decision["object_protocol_active"])

    def test_udp_video_timeout_returns_no_frame(self):
        cls, config = load_block("0_recibir_sumocam", "receiver.py", "UdpVideoReceiver")
        sock = Mock()
        sock.recvfrom.side_effect = socket.timeout
        with patch("socket.socket", return_value=sock):
            receiver = cls(**config)
            self.assertIsNone(receiver.read())

    def test_robot_wire_commands_and_shutdown_stop(self):
        cls, config = load_block("3_enviar_respuesta", "sender.py", "RobotCommandSender")
        sock = Mock()
        sock.recvfrom.side_effect = BlockingIOError
        with patch("socket.socket", return_value=sock), patch("time.sleep"):
            sender = cls(**config)
            with patch("time.monotonic", side_effect=[1.0, 2.0, 3.0, 4.0]):
                sender.send_action("forward")
                sender.send_action("curve_left", delta=8)
                sender.send_action("back_curve_left", delta=8)
                sender.stop()
            sender.close()
        self.assertEqual([call.args[0] for call in sock.sendto.call_args_list],
                         [b"F 20", b"LL 20 8", b"BL 20 8", b"S", b"S", b"S"])
        self.assertTrue(sock.close.called)


if __name__ == "__main__":
    unittest.main()
