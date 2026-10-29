#!/usr/bin/env python3
import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class MapViewerNode(Node):
    def __init__(self) -> None:
        super().__init__('map_viewer_node')

        self.declare_parameter('map_image_path', '/home/orangepi/Downloads')
        self.declare_parameter('window_name', 'Flight Map Viewer')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        self.declare_parameter('x_min_cm', 0.0)
        self.declare_parameter('x_max_cm', 300.0)
        self.declare_parameter('y_min_cm', 0.0)
        self.declare_parameter('y_max_cm', 400.0)

        self.declare_parameter('origin_is_bottom', True)
        self.declare_parameter('origin_is_right', True)
        self.declare_parameter('render_width', 900)
        self.declare_parameter('render_height', 700)
        self.declare_parameter('margin_px', 18)
        self.declare_parameter('draw_default_when_tf_missing', True)
        self.declare_parameter('show_hud_always', True)
        self.declare_parameter('debug_force_marker', True)
        self.declare_parameter('log_overlay_heartbeat', True)

        self.window_name = self.get_parameter('window_name').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        self.x_min = float(self.get_parameter('x_min_cm').value)
        self.x_max = float(self.get_parameter('x_max_cm').value)
        self.y_min = float(self.get_parameter('y_min_cm').value)
        self.y_max = float(self.get_parameter('y_max_cm').value)

        self.origin_is_bottom = bool(self.get_parameter('origin_is_bottom').value)
        self.origin_is_right = bool(self.get_parameter('origin_is_right').value)

        self.render_width = int(self.get_parameter('render_width').value)
        self.render_height = int(self.get_parameter('render_height').value)
        self.margin_px = int(self.get_parameter('margin_px').value)
        self.draw_default_when_tf_missing = bool(self.get_parameter('draw_default_when_tf_missing').value)
        self.show_hud_always = bool(self.get_parameter('show_hud_always').value)
        self.debug_force_marker = bool(self.get_parameter('debug_force_marker').value)
        self.log_overlay_heartbeat = bool(self.get_parameter('log_overlay_heartbeat').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        resolved_image_path = self._resolve_map_image_path(self.get_parameter('map_image_path').value)
        self.base_map = self._load_map_image(resolved_image_path)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.render_width, self.render_height)
        try:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 1)
        except Exception:
            pass
        self.frame_count = 0

        self.timer = self.create_timer(0.10, self._on_timer)
        self.get_logger().info('Map viewer started. Waiting for TF map -> base_link ...')

    def _resolve_map_image_path(self, raw_path: str) -> str:
        path = (raw_path or '').strip()
        if not path:
            return ''

        expanded = os.path.expanduser(path)
        if os.path.isdir(expanded):
            exts = ('.png', '.jpg', '.jpeg', '.bmp', '.webp')
            candidates = [
                os.path.join(expanded, name)
                for name in sorted(os.listdir(expanded))
                if name.lower().endswith(exts)
            ]
            if not candidates:
                self.get_logger().warn(f'No image found in directory: {expanded}')
                return ''
            selected = candidates[0]
            self.get_logger().info(f'Using map image: {selected}')
            return selected

        return expanded

    def _load_map_image(self, image_path: str) -> np.ndarray:
        img = None
        if image_path and os.path.exists(image_path):
            img = cv2.imread(image_path, cv2.IMREAD_COLOR)

        if img is None:
            self.get_logger().warn(
                f'Map image not found: {image_path}. Using generated fallback grid map.'
            )
            img = np.full((700, 900, 3), (90, 160, 90), dtype=np.uint8)
            for i in range(1, 9):
                x = int(i * img.shape[1] / 9)
                cv2.line(img, (x, 0), (x, img.shape[0]), (50, 80, 50), 1)
            for i in range(1, 7):
                y = int(i * img.shape[0] / 7)
                cv2.line(img, (0, y), (img.shape[1], y), (50, 80, 50), 1)
            cv2.circle(img, (img.shape[1] - 45, img.shape[0] - 45), 30, (0, 0, 255), -1)

        src_h, src_w = img.shape[:2]
        if src_w != self.render_width or src_h != self.render_height:
            self.get_logger().info(
                f'Resizing map image from {src_w}x{src_h} to {self.render_width}x{self.render_height}'
            )
            img = cv2.resize(img, (self.render_width, self.render_height), interpolation=cv2.INTER_AREA)

        return img

    def _clamp01(self, v: float) -> float:
        return max(0.0, min(1.0, v))

    def _xy_cm_to_pixel(self, x_cm: float, y_cm: float, w: int, h: int) -> tuple[int, int]:
        x_range = max(1e-6, self.x_max - self.x_min)
        y_range = max(1e-6, self.y_max - self.y_min)

        x_norm = self._clamp01((x_cm - self.x_min) / x_range)
        y_norm = self._clamp01((y_cm - self.y_min) / y_range)

        draw_w = max(1, w - 2 * self.margin_px)
        draw_h = max(1, h - 2 * self.margin_px)

        if self.origin_is_right:
            px = int(self.margin_px + (1.0 - y_norm) * draw_w)
        else:
            px = int(self.margin_px + y_norm * draw_w)

        if self.origin_is_bottom:
            py = int(self.margin_px + (1.0 - x_norm) * draw_h)
        else:
            py = int(self.margin_px + x_norm * draw_h)

        return px, py

    def _xy_cm_to_norm(self, x_cm: float, y_cm: float) -> tuple[float, float]:
        x_range = max(1e-6, self.x_max - self.x_min)
        y_range = max(1e-6, self.y_max - self.y_min)
        x_norm = self._clamp01((x_cm - self.x_min) / x_range)
        y_norm = self._clamp01((y_cm - self.y_min) / y_range)
        return x_norm, y_norm

    def _norm_to_widget_pixel(self, x_norm: float, y_norm: float, x0: int, y0: int, box_w: int, box_h: int) -> tuple[int, int]:
        if self.origin_is_right:
            px = int(x0 + (1.0 - y_norm) * box_w)
        else:
            px = int(x0 + y_norm * box_w)

        if self.origin_is_bottom:
            py = int(y0 + (1.0 - x_norm) * box_h)
        else:
            py = int(y0 + x_norm * box_h)

        return px, py

    def _draw_origin_marker(self, frame: np.ndarray, origin_px: int, origin_py: int, uav_px: int, uav_py: int) -> None:
        cv2.circle(frame, (origin_px, origin_py), 14, (0, 0, 0), -1)
        cv2.circle(frame, (origin_px, origin_py), 11, (0, 0, 255), -1)
        cv2.line(frame, (origin_px - 16, origin_py), (origin_px + 16, origin_py), (255, 255, 255), 2)
        cv2.line(frame, (origin_px, origin_py - 16), (origin_px, origin_py + 16), (255, 255, 255), 2)

        cv2.putText(frame, 'ORIGIN', (max(6, origin_px - 36), max(20, origin_py - 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3)
        cv2.putText(frame, 'ORIGIN', (max(6, origin_px - 36), max(20, origin_py - 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

        cv2.line(frame, (origin_px, origin_py), (uav_px, uav_py), (255, 255, 255), 1)

    def _yaw_from_quat(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _draw_aircraft(self, frame: np.ndarray, center: tuple[int, int], yaw_rad: float, pulse_radius: int) -> None:
        cx, cy = center
        body_radius = 14
        arrow_len = 40

        cv2.circle(frame, (cx, cy), pulse_radius + 3, (255, 255, 255), 3)
        cv2.circle(frame, (cx, cy), pulse_radius, (255, 0, 255), 2)

        cv2.circle(frame, (cx, cy), body_radius + 3, (0, 0, 0), -1)
        cv2.circle(frame, (cx, cy), body_radius, (0, 165, 255), -1)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

        tip_x = int(cx + arrow_len * math.cos(yaw_rad))
        tip_y = int(cy - arrow_len * math.sin(yaw_rad))
        cv2.arrowedLine(frame, (cx, cy), (tip_x, tip_y), (0, 0, 0), 4, tipLength=0.35)
        cv2.arrowedLine(frame, (cx, cy), (tip_x, tip_y), (255, 255, 255), 2, tipLength=0.35)

        label_pt = (max(0, cx + 14), max(18, cy - 14))
        cv2.putText(frame, 'UAV', label_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(frame, 'UAV', label_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    def _draw_hud(self, frame: np.ndarray, x_cm: float, y_cm: float, yaw_deg: float) -> None:
        text = f'x={x_cm:.1f}cm  y={y_cm:.1f}cm  yaw={yaw_deg:.1f}deg'
        cv2.rectangle(frame, (8, 8), (500, 42), (20, 20, 20), -1)
        cv2.putText(frame, text, (14, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 245, 245), 2)

    def _draw_debug_marker(self, frame: np.ndarray, w: int, h: int) -> None:
        tx = int(self.margin_px + (w - 2 * self.margin_px) * 0.5)
        ty = int(self.margin_px + (h - 2 * self.margin_px) * 0.5)
        blink = 18 + int(6.0 * (1.0 + math.sin(self.frame_count * 0.55)) / 2.0)
        cv2.circle(frame, (tx, ty), blink + 2, (0, 0, 0), 3)
        cv2.circle(frame, (tx, ty), blink, (0, 255, 0), 2)
        cv2.line(frame, (tx - 14, ty), (tx + 14, ty), (0, 255, 0), 2)
        cv2.line(frame, (tx, ty - 14), (tx, ty + 14), (0, 255, 0), 2)
        cv2.putText(frame, 'DEBUG MARKER', (max(8, tx - 60), max(20, ty - 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(frame, 'DEBUG MARKER', (max(8, tx - 60), max(20, ty - 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.rectangle(frame, (8, h - 40), (280, h - 8), (15, 15, 15), -1)
        cv2.putText(frame, 'MAP_VIEWER ACTIVE', (14, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 255, 80), 2)

    def _draw_position_widget(self, frame: np.ndarray, x_norm: float, y_norm: float, tf_ok: bool) -> None:
        h, w = frame.shape[:2]
        box_w = 180
        box_h = 180
        x0 = max(8, w - box_w - 12)
        y0 = 52
        x1 = x0 + box_w
        y1 = y0 + box_h

        cv2.rectangle(frame, (x0, y0), (x1, y1), (15, 15, 15), -1)
        cv2.rectangle(frame, (x0, y0), (x1, y1), (220, 220, 220), 1)

        for i in range(1, 4):
            gx = x0 + i * box_w // 4
            gy = y0 + i * box_h // 4
            cv2.line(frame, (gx, y0), (gx, y1), (70, 70, 70), 1)
            cv2.line(frame, (x0, gy), (x1, gy), (70, 70, 70), 1)

        px, py = self._norm_to_widget_pixel(x_norm, y_norm, x0, y0, box_w, box_h)

        origin_x, origin_y = self._norm_to_widget_pixel(0.0, 0.0, x0, y0, box_w, box_h)
        cv2.circle(frame, (origin_x, origin_y), 7, (0, 0, 255), -1)
        cv2.circle(frame, (origin_x, origin_y), 10, (255, 255, 255), 1)
        cv2.putText(frame, 'O', (origin_x + 10, max(y0 + 14, origin_y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)

        cv2.circle(frame, (px, py), 6, (0, 0, 0), -1)
        cv2.circle(frame, (px, py), 4, (0, 255, 255), -1)

        cv2.line(frame, (origin_x, origin_y), (px, py), (180, 180, 180), 1)

        status = 'TF OK' if tf_ok else 'TF LOST'
        status_color = (80, 220, 80) if tf_ok else (0, 100, 255)
        cv2.putText(frame, status, (x0 + 8, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.50, status_color, 1)
        cv2.putText(frame, 'Position Widget', (x0 + 8, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

    def _on_timer(self) -> None:
        self.frame_count += 1
        frame = self.base_map.copy()
        h, w = frame.shape[:2]
        tf_ok = False
        x_cm = self.x_min
        y_cm = self.y_min
        yaw = 0.0
        yaw_deg = 0.0

        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            x_cm = float(t.transform.translation.x) * 100.0
            y_cm = float(t.transform.translation.y) * 100.0

            q = t.transform.rotation
            yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
            yaw_deg = math.degrees(yaw)
            tf_ok = True
        except TransformException:
            cv2.putText(
                frame,
                f'Waiting TF: {self.map_frame} -> {self.robot_frame}',
                (14, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 255),
                2,
            )

        px = None
        py = None
        if tf_ok or self.draw_default_when_tf_missing:
            px, py = self._xy_cm_to_pixel(x_cm, y_cm, w, h)
            origin_px, origin_py = self._xy_cm_to_pixel(self.x_min, self.y_min, w, h)
            self._draw_origin_marker(frame, origin_px, origin_py, px, py)
            pulse = 22 + int(6.0 * (1.0 + math.sin(self.frame_count * 0.35)) / 2.0)
            self._draw_aircraft(frame, (px, py), yaw, pulse)

        if self.show_hud_always or tf_ok:
            self._draw_hud(frame, x_cm, y_cm, yaw_deg)

        if self.debug_force_marker:
            self._draw_debug_marker(frame, w, h)

        if not tf_ok:
            cv2.rectangle(frame, (8, 44), (320, 72), (20, 20, 20), -1)
            cv2.putText(frame, 'TF LOST - using fallback draw', (14, 63), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

        if self.log_overlay_heartbeat and self.frame_count % 20 == 0:
            if px is None or py is None:
                self.get_logger().info(f'[VIS] tf_ok={tf_ok} x={x_cm:.1f}cm y={y_cm:.1f}cm px=NA py=NA')
            else:
                self.get_logger().info(f'[VIS] tf_ok={tf_ok} x={x_cm:.1f}cm y={y_cm:.1f}cm px={px} py={py}')

        x_norm, y_norm = self._xy_cm_to_norm(x_cm, y_cm)
        self._draw_position_widget(frame, x_norm, y_norm, tf_ok)

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            self.get_logger().info('Closing map viewer by key press.')
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
