#!/usr/bin/env python3
"""ROS2 wrapper around the OpenCV QR decoder for the D题 shelf inventory task.

Frame source: V4L2 camera (default) or an image directory for hardware-free tests.
Publishes: /qr/result (JSON), /qr/code (Int32), /qr/debug_image, /laser/trigger.
"""

import json
import os
import sys
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from qr_decode import code_to_position, decode_qr, draw_results, parse_code  # noqa: E402

IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


class QrVisionNode(Node):
    """Decode QR codes and report goods codes plus shelf positions."""

    def __init__(self) -> None:
        super().__init__("qr_vision_node")

        self.declare_parameter("mode", "camera")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("rotate", 0)
        self.declare_parameter("image_dir", "")
        self.declare_parameter("fast_decode", True)
        self.declare_parameter("variant_every", 5)
        self.declare_parameter("confirm_frames", 2)
        self.declare_parameter("cooldown_sec", 2.0)
        self.declare_parameter("laser_pulse_sec", 0.5)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("serial_enabled", False)
        self.declare_parameter("serial_port", "")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("serial_frame_id", 0x41)

        self.mode = str(self.get_parameter("mode").value)
        self.device = str(self.get_parameter("device").value)
        self.rotate = int(self.get_parameter("rotate").value) % 360
        self.fast_decode = bool(self.get_parameter("fast_decode").value)
        self.variant_every = max(1, int(self.get_parameter("variant_every").value))
        self.confirm_frames = max(1, int(self.get_parameter("confirm_frames").value))
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.laser_pulse_sec = float(self.get_parameter("laser_pulse_sec").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.serial_frame_id = int(self.get_parameter("serial_frame_id").value) & 0xFF

        self.bridge = CvBridge()
        self.capture = None
        self.image_list = []
        self.image_index = 0
        self.frame_id = 0
        self.streak_code = None
        self.streak = 0
        self.last_published_code = None
        self.last_publish_time = 0.0
        self.laser_off_time = 0.0
        self.serial_fd = None
        self.last_open_attempt = 0.0

        self.result_pub = self.create_publisher(String, "/qr/result", 10)
        self.code_pub = self.create_publisher(Int32, "/qr/code", 10)
        self.laser_pub = self.create_publisher(Bool, "/laser/trigger", 10)
        self.debug_pub = (
            self.create_publisher(Image, "/qr/debug_image", 2) if self.publish_debug else None
        )

        if self.mode == "image_dir":
            self._load_images()
        else:
            self._open_camera()
        if bool(self.get_parameter("serial_enabled").value):
            self._open_serial()

        fps = float(self.get_parameter("fps").value)
        self.timer = self.create_timer(1.0 / max(1.0, fps), self.on_timer)
        self.get_logger().info(
            f"qr_vision_node started: mode={self.mode} fast={self.fast_decode} "
            f"confirm={self.confirm_frames} debug_image={self.publish_debug}"
        )

    # ---------------- frame sources ----------------

    def _load_images(self) -> None:
        directory = Path(str(self.get_parameter("image_dir").value))
        if not directory.is_dir():
            self.get_logger().error(f"image_dir not found: {directory}")
            return
        self.image_list = sorted(
            path for path in directory.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES
        )
        self.get_logger().info(f"loaded {len(self.image_list)} test images from {directory}")

    def _open_camera(self) -> bool:
        self._release_camera()
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            return False
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("width").value))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("height").value))
        capture.set(cv2.CAP_PROP_FPS, float(self.get_parameter("fps").value))
        # BUFFERSIZE=1 会让帧率从 30fps 掉到 15fps（实测），用 3 兼顾帧率与延迟
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 3)
        self.capture = capture
        self.get_logger().info(f"camera opened: {self.device}")
        return True

    def _release_camera(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def _open_serial(self) -> None:
        port = str(self.get_parameter("serial_port").value)
        if not port:
            self.get_logger().warn("serial_enabled=true but serial_port is empty; skipping serial output")
            return
        try:
            import termios

            baud = int(self.get_parameter("serial_baud").value)
            self.serial_fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
            attrs = termios.tcgetattr(self.serial_fd)
            attrs[4] = attrs[5] = termios.B115200 if baud == 115200 else termios.B921600
            termios.tcsetattr(self.serial_fd, termios.TCSANOW, attrs)
            self.get_logger().info(f"serial output enabled: {port} @ {baud}")
        except Exception as error:  # noqa: BLE001 - report and keep vision running
            self.serial_fd = None
            self.get_logger().error(f"failed to open serial {port}: {error}")

    def _send_serial(self, code: int) -> None:
        if self.serial_fd is None:
            return
        face = (code - 1) // 6
        slot = (code - 1) % 6 + 1
        frame = self._build_frame(bytes([code, face, slot]))
        try:
            os.write(self.serial_fd, frame)
        except OSError as error:
            self.get_logger().error(f"serial write failed: {error}")

    def _build_frame(self, payload: bytes) -> bytes:
        frame = bytearray([0xAA, 0xFF, self.serial_frame_id, len(payload)])
        frame.extend(payload)
        checksum = 0
        add = 0
        for byte in frame:
            checksum = (checksum + byte) & 0xFF
            add = (add + checksum) & 0xFF
        frame.extend([checksum, add])
        return bytes(frame)

    # ---------------- main loop ----------------

    def _grab_frame(self):
        if self.mode == "image_dir":
            if not self.image_list:
                return None
            # Hold each image for confirm_frames frames so the multi-frame
            # confirmation logic behaves like a continuous video stream.
            index = (self.frame_id // self.confirm_frames) % len(self.image_list)
            path = self.image_list[index]
            self.image_index += 1
            return cv2.imread(str(path))

        if self.capture is None:
            now = time.monotonic()
            if now - self.last_open_attempt > 2.0:
                self.last_open_attempt = now
                if not self._open_camera():
                    self.get_logger().error(f"camera not available: {self.device}")
            return None

        ok, frame = self.capture.read()
        return frame if ok else None

    def _rotate(self, frame):
        if self.rotate == 90:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if self.rotate == 180:
            return cv2.rotate(frame, cv2.ROTATE_180)
        if self.rotate == 270:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame

    def on_timer(self) -> None:
        frame = self._grab_frame()
        if frame is None:
            self._update_laser()
            return

        frame = self._rotate(frame)
        self.frame_id += 1

        results = decode_qr(frame, fast=self.fast_decode)
        used_variants = False
        if not results and not self.fast_decode:
            results = decode_qr(frame, fast=False)
            used_variants = True
        elif not results and self.fast_decode and self.frame_id % self.variant_every == 0:
            results = decode_qr(frame, fast=False)
            used_variants = True

        codes = [code for code in (parse_code(item["text"]) for item in results) if code]

        if self.debug_pub is not None:
            annotated = draw_results(frame, results)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))

        if not codes:
            if results and self.frame_id % 15 == 0:
                self.get_logger().info(
                    f"decoded but not a 1~24 code: {[item['text'] for item in results]}"
                )
            self.streak = 0
            self.streak_code = None
            self._update_laser()
            return

        code = codes[0]
        if code == self.streak_code:
            self.streak += 1
        else:
            self.streak_code = code
            self.streak = 1

        if self.streak >= self.confirm_frames:
            self._publish(code, results[0], used_variants)
        self._update_laser()

    # ---------------- outputs ----------------

    def _publish(self, code: int, result, used_variants: bool) -> None:
        now = time.monotonic()
        same = code == self.last_published_code
        if same and now - self.last_publish_time < self.cooldown_sec:
            return

        position = code_to_position(code)
        payload = {
            "code": code,
            "position": position,
            "text": result["text"],
            "center": result["center"],
            "points": result["points"],
            "frame": self.frame_id,
            "variants": used_variants,
            "stamp": time.time(),
        }
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.code_pub.publish(Int32(data=code))
        self._send_serial(code)

        self.last_published_code = code
        self.last_publish_time = now
        self.streak = 0
        self.streak_code = None

        self.laser_pub.publish(Bool(data=True))
        self.laser_off_time = now + self.laser_pulse_sec

        self.get_logger().info(
            f"code={code} position={position} center={result['center']} "
            f"frame={self.frame_id} variants={used_variants} laser={self.laser_pulse_sec}s"
        )

    def _update_laser(self) -> None:
        if self.laser_off_time and time.monotonic() >= self.laser_off_time:
            self.laser_pub.publish(Bool(data=False))
            self.laser_off_time = 0.0

    def destroy_node(self):
        """Release hardware before shutdown."""
        self._release_camera()
        if self.serial_fd is not None:
            os.close(self.serial_fd)
            self.serial_fd = None
        return super().destroy_node()


def main(args=None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    node = QrVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGTERM (systemd stop / `timeout`) already shuts the context down.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
