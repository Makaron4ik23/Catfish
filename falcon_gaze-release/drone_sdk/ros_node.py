import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import os
WORLD_NAME = os.environ.get("WORLD_NAME", "baylands_custom")
CAMERA_TOPIC = f'/world/{WORLD_NAME}/model/x500_mono_cam_{{}}/link/mono_cam/base_link/sensor/camera_sensor/image'

LED_TOPIC = '/model/x500_mono_cam_{}/led_cmd'


class DroneROSNode(Node):

    def __init__(self, drone_id: int):
        super().__init__(f'drone_{drone_id}_sdk_node')
        self._drone_id = drone_id
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None

        cam_topic = CAMERA_TOPIC.format(drone_id)
        self._cam_sub = self.create_subscription(
            Image, cam_topic, self._image_cb, 10
        )

        led_topic = LED_TOPIC.format(drone_id)
        self._led_pub = self.create_publisher(String, led_topic, 10)

        # High-speed telemetry channel setup
        self._telemetry_pub = self.create_publisher(String, f'/drone_{drone_id}/telemetry', 10)
        self._target_telemetry_lock = threading.Lock()
        self._target_telemetry_by_id = {}
        self._default_target_id = drone_id - 1 if drone_id > 0 else None
        self._telemetry_subs = []

        target_ids = []
        if self._default_target_id is not None:
            target_ids.append(self._default_target_id)
        if drone_id > 1:
            target_ids.append(0)
        for target_id in sorted(set(target_ids)):
            target_topic = f'/drone_{target_id}/telemetry'
            self._telemetry_subs.append(
                self.create_subscription(
                    String,
                    target_topic,
                    lambda msg, idx=target_id: self._telemetry_cb(idx, msg),
                    10,
                )
            )

        self._spin_thread: Optional[threading.Thread] = None
        self._spinning = False

    def _telemetry_cb(self, target_id: int, msg: String) -> None:
        try:
            import json
            data = json.loads(msg.data)
            data["_received_monotonic"] = time.monotonic()
            with self._target_telemetry_lock:
                self._target_telemetry_by_id[target_id] = data
        except Exception:
            pass

    def publish_telemetry(self, value: str) -> None:
        msg = String()
        msg.data = value
        self._telemetry_pub.publish(msg)

    def target_telemetry(self, target_id: Optional[int] = None, max_age_s: float = 1.0) -> Optional[dict]:
        selected_id = self._default_target_id if target_id is None else target_id
        if selected_id is None:
            return None
        with self._target_telemetry_lock:
            data = self._target_telemetry_by_id.get(selected_id)
            if data is None:
                return None
            age = time.monotonic() - data.get("_received_monotonic", 0.0)
            if age > max_age_s:
                return None
            return dict(data)

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._latest_frame = frame
        except Exception:
            pass

    def frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._latest_frame

    def publish_led(self, value: str) -> None:
        msg = String()
        msg.data = value
        self._led_pub.publish(msg)

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.001)

    def start_spin(self) -> None:
        if self._spinning:
            return
        self._spinning = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def stop_spin(self) -> None:
        self._spinning = False

    def _spin_loop(self) -> None:
        while self._spinning and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
