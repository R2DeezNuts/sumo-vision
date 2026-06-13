"""Procesado de imagen para navegacion entre lineas y deteccion de objetos.

El modulo recibe frames BGR de OpenCV, detecta lineas oscuras del carril,
estima el centro navegable y localiza objetos oscuros que puedan invadir la
trayectoria. El resultado es un diccionario estructurado con accion sugerida,
confianza, offset, imagenes de depuracion y objetos detectados.
"""

from __future__ import annotations

import cv2
import numpy as np

ACTION_FORWARD = "forward"
ACTION_LEFT = "left"
ACTION_RIGHT = "right"
ACTION_STOP = "stop"

KERNEL_3 = np.ones((3, 3), np.uint8)
KERNEL_5 = np.ones((5, 5), np.uint8)


class ImageProcessor:
    """Pipeline de vision basado en umbrales, morfologia y contornos."""

    def __init__(
        self,
        width=800,
        roi_top=0.6,
        roi_bottom=1.0,
        contrast_threshold=18,
        absolute_value_threshold=120,
        min_area=1000,
        confidence_stop=0.18,
        object_detect_enabled=True,
        object_min_area=1200,
        object_dark_value_threshold=70,
        object_dark_value_threshold_max=95,
        use_perspective=True,
        src_norm=None,
    ):
        """Inicializa solo los parametros editables desde ``config.json``.

        Los ajustes finos que no se modifican habitualmente se fijan aqui como
        constantes internas para mantener limpia la configuracion de entrega.
        """
        self.width = int(width)
        self.roi_top = float(np.clip(roi_top, 0.0, 0.95))
        self.roi_bottom = float(np.clip(roi_bottom, 0.05, 1.0))
        self.min_area = int(max(20, min_area))
        self.confidence_stop = float(np.clip(confidence_stop, 0.0, 1.0))
        self.use_perspective = bool(use_perspective)
        self.contrast_threshold = int(np.clip(contrast_threshold, 1, 80))
        self.absolute_value_threshold = int(np.clip(absolute_value_threshold, 20, 255))

        self.full_turn_offset = 100.0
        self.min_turn_percent = 6
        self.turn_deadband_px = 15
        self.turn_confirm = 2
        self.steering_alpha = 0.45
        self.contrast_blur_kernel = 61
        self.min_two_line_width_ratio = 0.22
        self.single_line_side_lock_frames = 6

        self.box_detect_enabled = bool(object_detect_enabled)
        self.box_black_value_threshold = int(np.clip(object_dark_value_threshold, 20, 140))
        self.box_black_value_threshold_max = int(
            np.clip(object_dark_value_threshold_max, self.box_black_value_threshold, 180)
        )
        self.box_min_area = int(max(100, object_min_area))
        self.box_dark_percentile = 10.0
        self.box_max_saturation = 140
        self.box_max_frame_area_ratio = 0.08
        self.box_min_fill_ratio = 0.3
        self.box_min_aspect_ratio = 0.3
        self.box_max_aspect_ratio = 4.0
        self.box_border_margin_px = 2
        self.box_merge_min_x_overlap_ratio = 0.35
        self.box_merge_max_vertical_gap_ratio = 0.45
        self.box_line_mask_padding_px = 12

        if src_norm is None:
            src_norm = [
                [0.0, 0.0],
                [0.0, 0.9],
                [1.0, 0.0],
                [1.0, 0.9],
            ]
        src_norm = np.asarray(src_norm, dtype=np.float32)
        if src_norm.shape != (4, 2):
            raise ValueError("src_norm debe tener forma 4x2")
        self.src_norm = np.clip(src_norm, 0.0, 1.0)

        self._pending_action = None
        self._pending_count = 0
        self._smoothed_steering = 0.0
        self._lane_width_px = None
        self._last_single_line_side = None
        self._single_side_stable_count = 0

    def _resize_keep_aspect(self, frame):
        if self.width <= 0 or frame.shape[1] == self.width:
            return frame
        scale = self.width / frame.shape[1]
        height = max(1, int(frame.shape[0] * scale))
        return cv2.resize(frame, (self.width, height), interpolation=cv2.INTER_AREA)

    def _invalid_result(self, frame, reason):
        out = frame if frame is not None else np.zeros((480, max(1, self.width), 3), dtype=np.uint8)
        debug_mask = np.zeros(out.shape[:2], dtype=np.uint8)
        debug_bird = out.copy()
        cv2.putText(debug_bird, f"bird: {reason}", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (0, 0, 255), 2, cv2.LINE_AA)
        return self._result(
            out, ACTION_STOP, 0, 0.0, None, reason, [], debug_mask,
            cv2.cvtColor(debug_mask, cv2.COLOR_GRAY2BGR), debug_bird,
        )

    def _result(
        self, frame, action, steering_percent, confidence, offset_px, reason,
        objects, debug_mask, debug_roi, debug_bird, single_line_lower_half=None,
    ):
        return {
            "frame": frame,
            "action": action,
            "steering_percent": int(steering_percent),
            "confidence": float(confidence),
            "offset_px": None if offset_px is None else float(offset_px),
            "reason": reason,
            "image_width": int(frame.shape[1]),
            "image_height": int(frame.shape[0]),
            "objects": objects,
            "boxes": objects,
            "debug_mask": debug_mask,
            "debug_roi": debug_roi,
            "debug_bird": debug_bird,
            **({} if single_line_lower_half is None else {"single_line_lower_half": bool(single_line_lower_half)}),
        }

    def _stabilize_action(self, raw_action):
        if raw_action in (ACTION_FORWARD, ACTION_STOP):
            self._pending_action = None
            self._pending_count = 0
            return raw_action

        if self._pending_action != raw_action:
            self._pending_action = raw_action
            self._pending_count = 1
            return ACTION_FORWARD

        self._pending_count += 1
        return raw_action if self._pending_count >= self.turn_confirm else ACTION_FORWARD

    def _build_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        value = hsv[:, :, 2]

        background = cv2.GaussianBlur(
            value,
            (self.contrast_blur_kernel, self.contrast_blur_kernel),
            0,
        )
        dark_contrast = cv2.subtract(background, value)
        _, contrast_mask = cv2.threshold(
            dark_contrast,
            self.contrast_threshold,
            255,
            cv2.THRESH_BINARY,
        )

        lower = np.array([0, 0, 0], dtype=np.uint8)
        upper = np.array([180, 255, self.absolute_value_threshold], dtype=np.uint8)
        absolute_mask = cv2.inRange(hsv, lower, upper)

        mask = cv2.bitwise_or(contrast_mask, absolute_mask)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL_3, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_5, iterations=2)
        return cv2.dilate(mask, KERNEL_5, iterations=1)

    def _stable_single_line_side(self, side):
        if side not in ("left", "right"):
            self._last_single_line_side = None
            self._single_side_stable_count = 0
            return None

        if self._last_single_line_side is None:
            self._last_single_line_side = side
            self._single_side_stable_count = 1
            return side

        if side == self._last_single_line_side:
            self._single_side_stable_count += 1
            return side

        if self._single_side_stable_count < self.single_line_side_lock_frames:
            self._single_side_stable_count += 1
            return self._last_single_line_side

        self._last_single_line_side = side
        self._single_side_stable_count = 1
        return side

    def _confidence(self, area, roi_area, offset_px, width):
        area_score = float(np.clip(area / max(1.0, 0.08 * roi_area), 0.0, 1.0))
        center_score = 1.0 - float(np.clip(abs(offset_px) / max(1.0, width * 0.5), 0.0, 1.0))
        return float(np.clip(0.65 * area_score + 0.35 * center_score, 0.0, 1.0))

    def _raw_action_from_offset(self, offset_px, confidence):
        if confidence < self.confidence_stop:
            return ACTION_STOP, 0

        steering = int(np.clip((abs(offset_px) / self.full_turn_offset) * 100.0, 0, 100))
        if 0 < steering < self.min_turn_percent:
            steering = self.min_turn_percent

        self._smoothed_steering = (
            self.steering_alpha * float(steering)
            + (1.0 - self.steering_alpha) * self._smoothed_steering
        )
        steering_smoothed = int(np.clip(round(self._smoothed_steering), 0, 100))

        if abs(offset_px) <= self.turn_deadband_px:
            return ACTION_FORWARD, 0

        if offset_px < 0:
            return ACTION_LEFT, max(self.min_turn_percent, steering_smoothed)
        return ACTION_RIGHT, max(self.min_turn_percent, steering_smoothed)

    def _scaled_src_points(self, width, height):
        pts = self.src_norm.copy()
        pts[:, 0] *= float(max(1, width - 1))
        pts[:, 1] *= float(max(1, height - 1))
        return pts.astype(np.float32)

    def _warp_invalid(self, bird):
        gray = cv2.cvtColor(bird, cv2.COLOR_BGR2GRAY)
        return int(gray.max()) - int(gray.min()) < 8

    def _perspective_prepare(self, frame):
        h, w = frame.shape[:2]
        identity = np.eye(3, dtype=np.float32)
        if not self.use_perspective:
            return frame.copy(), identity, identity, False

        src = self._scaled_src_points(w, h)
        dst = np.float32([
            [0, 0],
            [0, h - 1],
            [w - 1, 0],
            [w - 1, h - 1],
        ])

        matrix = cv2.getPerspectiveTransform(src, dst)
        inv_matrix = cv2.getPerspectiveTransform(dst, src)
        bird = cv2.warpPerspective(frame, matrix, (w, h))
        if self._warp_invalid(bird):
            return frame.copy(), identity, identity, True
        return bird, matrix, inv_matrix, False

    def _bird_point_to_frame(self, inv_matrix, x, y):
        pt = np.array([[[float(x), float(y)]]], dtype=np.float32)
        frame_pt = cv2.perspectiveTransform(pt, inv_matrix)
        return int(frame_pt[0, 0, 0]), int(frame_pt[0, 0, 1])

    def _detect_objects(self, frame):
        if not self.box_detect_enabled:
            return []

        height, width = frame.shape[:2]
        frame_area = float(max(1, height * width))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        value = hsv[:, :, 2]
        sat = hsv[:, :, 1]

        adaptive_thr = int(np.percentile(value, self.box_dark_percentile))
        dark_thr = int(np.clip(adaptive_thr, self.box_black_value_threshold, self.box_black_value_threshold_max))
        mask = cv2.bitwise_and(
            (value <= dark_thr).astype(np.uint8) * 255,
            (sat <= self.box_max_saturation).astype(np.uint8) * 255,
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL_3, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_3, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.box_min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            m = self.box_border_margin_px
            if x <= m or y <= m or (x + bw) >= (width - m) or (y + bh) >= (height - m):
                continue
            if area / frame_area > self.box_max_frame_area_ratio:
                continue

            rect_area = float(max(1, bw * bh))
            fill_ratio = area / rect_area
            aspect_ratio = float(bw) / float(max(1, bh))
            if (
                fill_ratio < self.box_min_fill_ratio
                or not (self.box_min_aspect_ratio <= aspect_ratio <= self.box_max_aspect_ratio)
            ):
                continue

            objects.append(
                self._make_object(x, y, bw, bh, area, frame.shape, merged_count=1)
            )

        objects = self._merge_stacked_objects(objects, frame.shape)
        objects.sort(key=lambda obj: obj["area"], reverse=True)
        return objects

    def _make_object(self, x, y, w, h, area, frame_shape, merged_count=1):
        frame_area = float(max(1, frame_shape[0] * frame_shape[1]))
        rect_area = float(max(1, w * h))
        return {
            "kind": "object",
            "x": int(x),
            "y": int(y),
            "w": int(w),
            "h": int(h),
            "area": float(area),
            "frame_area_ratio": float(area / frame_area),
            "fill_ratio": float(area / rect_area),
            "aspect_ratio": float(w) / float(max(1, h)),
            "merged_count": int(max(1, merged_count)),
        }

    def _objects_stackable(self, a, b):
        ax1, ay1 = int(a["x"]), int(a["y"])
        ax2, ay2 = ax1 + int(a["w"]), ay1 + int(a["h"])
        bx1, by1 = int(b["x"]), int(b["y"])
        bx2, by2 = bx1 + int(b["w"]), by1 + int(b["h"])

        x_overlap = min(ax2, bx2) - max(ax1, bx1)
        if x_overlap <= 0:
            return False

        min_width = float(max(1, min(int(a["w"]), int(b["w"]))))
        if x_overlap / min_width < self.box_merge_min_x_overlap_ratio:
            return False

        vertical_gap = max(0, max(ay1, by1) - min(ay2, by2))
        height_ref = float(max(1, min(int(a["h"]), int(b["h"]))))
        max_gap = max(4.0, self.box_merge_max_vertical_gap_ratio * height_ref)
        return float(vertical_gap) <= max_gap

    def _merge_two_objects(self, a, b, frame_shape):
        x1 = min(int(a["x"]), int(b["x"]))
        y1 = min(int(a["y"]), int(b["y"]))
        x2 = max(int(a["x"]) + int(a["w"]), int(b["x"]) + int(b["w"]))
        y2 = max(int(a["y"]) + int(a["h"]), int(b["y"]) + int(b["h"]))
        area = float(a.get("area", 0.0)) + float(b.get("area", 0.0))
        merged_count = int(a.get("merged_count", 1)) + int(b.get("merged_count", 1))
        return self._make_object(x1, y1, x2 - x1, y2 - y1, area, frame_shape, merged_count=merged_count)

    def _merge_stacked_objects(self, objects, frame_shape):
        if len(objects) < 2:
            return objects

        merged = list(objects)
        changed = True
        while changed:
            changed = False
            for i in range(len(merged)):
                for j in range(i + 1, len(merged)):
                    if not self._objects_stackable(merged[i], merged[j]):
                        continue
                    merged[i] = self._merge_two_objects(merged[i], merged[j], frame_shape)
                    del merged[j]
                    changed = True
                    break
                if changed:
                    break
        return merged

    def _remove_objects_from_line_mask(self, mask, objects, perspective_matrix):
        if not objects:
            return mask

        h, w = mask.shape[:2]
        out = mask.copy()
        pad = self.box_line_mask_padding_px
        for obj in objects:
            x1 = int(np.clip(int(obj["x"]) - pad, 0, w - 1))
            y1 = int(np.clip(int(obj["y"]) - pad, 0, h - 1))
            x2 = int(np.clip(int(obj["x"]) + int(obj["w"]) + pad, 0, w - 1))
            y2 = int(np.clip(int(obj["y"]) + int(obj["h"]) + pad, 0, h - 1))
            pts = np.array(
                [[
                    [x1, y1],
                    [x2, y1],
                    [x2, y2],
                    [x1, y2],
                ]],
                dtype=np.float32,
            )
            pts = cv2.perspectiveTransform(pts, perspective_matrix)[0]
            pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
            pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
            cv2.fillConvexPoly(out, np.round(pts).astype(np.int32), 0)

        return out

    def _contour_anchor(self, contour):
        pts = contour[:, 0, :]
        if pts.size == 0:
            return None
        bottom_y = int(np.max(pts[:, 1]))
        top_y = int(np.min(pts[:, 1]))
        height = max(1, bottom_y - top_y)
        band_top = bottom_y - max(8, int(0.25 * height))
        bottom_pts = pts[pts[:, 1] >= band_top]
        if len(bottom_pts) == 0:
            bottom_pts = pts
        cx = int(np.median(bottom_pts[:, 0]))
        cy = int(np.median(bottom_pts[:, 1]))
        return cx, cy, bottom_y

    def _line_candidate(self, contour, roi_top):
        contour = contour.copy()
        contour[:, 0, 1] += roi_top
        area = float(cv2.contourArea(contour))
        anchor = None if area < self.min_area else self._contour_anchor(contour)
        if anchor is None:
            return None
        cx, cy, bottom_y = anchor
        return {"contour": contour, "area": area, "cx": cx, "cy": cy, "bottom_y": bottom_y}

    def _line_candidates(self, contours, roi_top, image_center_x):
        left = right = None
        for contour in contours:
            candidate = self._line_candidate(contour, roi_top)
            if candidate is None:
                continue
            side = "left" if candidate["cx"] < image_center_x else "right"
            current = left if side == "left" else right
            better = current is None or (candidate["bottom_y"], candidate["area"]) > (
                current["bottom_y"], current["area"]
            )
            if side == "left" and better:
                left = candidate
            elif better:
                right = candidate
        return left, right

    def _lane_from_candidates(self, left, right, width):
        two_lines = left is not None and right is not None
        if two_lines and abs(float(right["cx"] - left["cx"])) < self.min_two_line_width_ratio * float(width):
            if left["area"] >= right["area"]:
                right = None
            else:
                left = None
            two_lines = False

        if two_lines:
            self._update_lane_width(abs(float(right["cx"] - left["cx"])), width)
            self._last_single_line_side = None
            self._single_side_stable_count = 0
            return (
                left, right, True,
                0.5 * (left["cx"] + right["cx"]),
                int(max(left["cy"], right["cy"])),
                left["area"] + right["area"],
                "two_lines",
            )

        candidate = left if left is not None else right
        detected_side = "left" if left is not None else "right"
        chosen_side = self._stable_single_line_side(detected_side)
        sign = 1.0 if chosen_side == "left" else -1.0
        return (
            left, right, False,
            float(candidate["cx"] + sign * 0.5 * self._estimate_lane_width(width)),
            int(candidate["cy"]),
            candidate["area"],
            f"one_line_{chosen_side}",
        )

    def _estimate_lane_width(self, width):
        if self._lane_width_px is None:
            return max(40.0, width * 0.45)
        return float(np.clip(self._lane_width_px, 40.0, width * 0.95))

    def _update_lane_width(self, width_px, image_width):
        width_px = float(np.clip(width_px, 40.0, image_width * 0.95))
        if self._lane_width_px is None:
            self._lane_width_px = width_px
        else:
            self._lane_width_px = 0.8 * self._lane_width_px + 0.2 * width_px

    def process(self, frame):
        """Procesa un frame y devuelve decision visual y vistas de diagnostico.

        Args:
            frame: Imagen BGR leida de la ESP32-CAM.

        Returns:
            Diccionario con ``action``, ``steering_percent``, ``confidence``,
            ``offset_px``, ``reason``, ``objects`` y frames de depuracion.
        """
        if frame is None:
            return self._invalid_result(None, "frame_none")
        if not isinstance(frame, np.ndarray):
            return self._invalid_result(None, "frame_not_ndarray")
        if frame.ndim != 3 or frame.shape[2] != 3:
            return self._invalid_result(None, "frame_shape_invalid")
        if frame.shape[0] < 8 or frame.shape[1] < 8:
            return self._invalid_result(frame, "frame_too_small")

        work = self._resize_keep_aspect(frame)
        h, w = work.shape[:2]

        bird, perspective_matrix, inv_matrix, perspective_fallback = self._perspective_prepare(work)
        bird_debug = bird.copy()

        roi_top = int(np.clip(self.roi_top * h, 0, h - 1))
        roi_bottom = int(np.clip(self.roi_bottom * h, roi_top + 1, h))
        objects = self._detect_objects(work)
        mask = self._build_mask(bird)
        mask = self._remove_objects_from_line_mask(mask, objects, perspective_matrix)
        if roi_top > 0:
            mask[:roi_top, :] = 0
        if roi_bottom < h:
            mask[roi_bottom:, :] = 0

        roi = mask[roi_top:roi_bottom, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        overlay = work.copy()
        image_center_x = w // 2
        cv2.line(overlay, (image_center_x, 0), (image_center_x, h - 1), (255, 0, 0), 1)
        cv2.line(bird_debug, (image_center_x, 0), (image_center_x, h - 1), (255, 0, 0), 1)
        cv2.line(bird_debug, (0, roi_top), (w - 1, roi_top), (64, 64, 255), 1)
        cv2.line(bird_debug, (0, roi_bottom - 1), (w - 1, roi_bottom - 1), (64, 64, 255), 1)
        for obj in objects:
            p1 = (obj["x"], obj["y"])
            p2 = (obj["x"] + obj["w"], obj["y"] + obj["h"])
            cv2.rectangle(overlay, p1, p2, (0, 165, 255), 2)
            cv2.putText(
                overlay,
                f"obj {int(obj['area'])}"
                + (f" x{obj['merged_count']}" if int(obj.get("merged_count", 1)) > 1 else ""),
                (obj["x"], max(12, obj["y"] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 165, 255),
                1,
                cv2.LINE_AA,
            )
        if perspective_fallback:
            cv2.putText(
                bird_debug,
                "Perspective fallback",
                (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.66,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        left_candidate, right_candidate = self._line_candidates(contours, roi_top, image_center_x)

        if left_candidate is None and right_candidate is None:
            cv2.putText(overlay, "No line", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (0, 0, 255), 2, cv2.LINE_AA)
            return self._result(
                overlay, ACTION_STOP, 0, 0.0, None, "no_lines", objects,
                mask, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), bird_debug,
            )
        (
            left_candidate, right_candidate, two_lines, lane_center_x_bird,
            ref_y_bird, area_sum, reason,
        ) = self._lane_from_candidates(left_candidate, right_candidate, w)
        left_contour = left_candidate["contour"] if left_candidate is not None else None
        right_contour = right_candidate["contour"] if right_candidate is not None else None
        left_cx = left_candidate["cx"] if left_candidate is not None else None
        right_cx = right_candidate["cx"] if right_candidate is not None else None
        left_cy = left_candidate["cy"] if left_candidate is not None else None
        right_cy = right_candidate["cy"] if right_candidate is not None else None

        lane_center_x_bird = float(np.clip(lane_center_x_bird, 0, w - 1))
        ref_y_bird = int(np.clip(ref_y_bird, roi_top, roi_bottom - 1))
        roi_mid_y = roi_top + 0.5 * float(roi_bottom - roi_top)
        single_line_lower_half = bool((not two_lines) and ref_y_bird >= roi_mid_y)

        if self.use_perspective and not perspective_fallback:
            cx_frame, cy_frame = self._bird_point_to_frame(inv_matrix, lane_center_x_bird, ref_y_bird)
            offset_px = float(cx_frame - image_center_x)
            roi_top_left_frame = self._bird_point_to_frame(inv_matrix, 0, roi_top)
            roi_top_right_frame = self._bird_point_to_frame(inv_matrix, w - 1, roi_top)
            roi_bottom_left_frame = self._bird_point_to_frame(inv_matrix, 0, roi_bottom - 1)
            roi_bottom_right_frame = self._bird_point_to_frame(inv_matrix, w - 1, roi_bottom - 1)
            cv2.line(overlay, roi_top_left_frame, roi_top_right_frame, (64, 64, 255), 1)
            cv2.line(overlay, roi_bottom_left_frame, roi_bottom_right_frame, (64, 64, 255), 1)
        else:
            cx_frame, cy_frame = int(lane_center_x_bird), ref_y_bird
            offset_px = float(lane_center_x_bird - image_center_x)
            cv2.line(overlay, (0, roi_top), (w - 1, roi_top), (64, 64, 255), 1)
            cv2.line(overlay, (0, roi_bottom - 1), (w - 1, roi_bottom - 1), (64, 64, 255), 1)

        roi_area = float(max(1, roi_bottom - roi_top) * w)
        confidence = self._confidence(area_sum, roi_area, offset_px, w)
        if not two_lines:
            confidence *= 0.75
        raw_action, steering_percent = self._raw_action_from_offset(offset_px, confidence)
        if not two_lines and confidence >= self.confidence_stop:
            raw_action = ACTION_FORWARD
            steering_percent = 0
        action = self._stabilize_action(raw_action)

        cv2.circle(overlay, (cx_frame, cy_frame), 5, (0, 255, 0), -1)
        cv2.line(overlay, (image_center_x, cy_frame), (cx_frame, cy_frame), (0, 255, 0), 2)
        if left_contour is not None:
            cv2.drawContours(bird_debug, [left_contour], -1, (255, 255, 0), 2)
            cv2.circle(bird_debug, (left_cx, left_cy), 5, (255, 255, 0), -1)
        if right_contour is not None:
            cv2.drawContours(bird_debug, [right_contour], -1, (0, 255, 255), 2)
            cv2.circle(bird_debug, (right_cx, right_cy), 5, (0, 255, 255), -1)
        cv2.circle(bird_debug, (int(lane_center_x_bird), ref_y_bird), 5, (0, 255, 0), -1)
        cv2.line(bird_debug, (image_center_x, ref_y_bird), (int(lane_center_x_bird), ref_y_bird), (0, 255, 0), 2)

        roi_debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if left_contour is not None:
            cv2.drawContours(roi_debug, [left_contour], -1, (255, 255, 0), 2)
            cv2.circle(roi_debug, (left_cx, left_cy), 5, (255, 255, 0), -1)
        if right_contour is not None:
            cv2.drawContours(roi_debug, [right_contour], -1, (0, 255, 255), 2)
            cv2.circle(roi_debug, (right_cx, right_cy), 5, (0, 255, 255), -1)
        cv2.circle(roi_debug, (int(lane_center_x_bird), ref_y_bird), 5, (0, 255, 0), -1)
        cv2.line(roi_debug, (image_center_x, roi_top), (image_center_x, roi_bottom - 1), (255, 0, 0), 1)
        cv2.line(roi_debug, (0, roi_top), (w - 1, roi_top), (64, 64, 255), 1)
        cv2.line(roi_debug, (0, int(round(roi_mid_y))), (w - 1, int(round(roi_mid_y))), (128, 128, 255), 1)
        cv2.line(roi_debug, (0, roi_bottom - 1), (w - 1, roi_bottom - 1), (64, 64, 255), 1)

        if perspective_fallback:
            reason = f"{reason}_warp_fallback"
        if action == ACTION_STOP:
            reason = "low_confidence" if confidence < self.confidence_stop else "stabilizing"

        label = (
            f"act={action} steer={steering_percent}% conf={confidence:.2f} "
            f"off={offset_px:+.1f}px {reason}"
        )
        cv2.rectangle(overlay, (0, 0), (w, 34), (0, 0, 0), thickness=-1)
        cv2.putText(overlay, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

        return self._result(
            overlay, action, steering_percent, confidence, offset_px, reason,
            objects, mask, roi_debug, bird_debug, single_line_lower_half,
        )
