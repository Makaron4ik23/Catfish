import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


CAMERA_TOPIC = '/world/baylands_custom/model/x500_mono_cam_{}/link/mono_cam/base_link/sensor/camera_sensor/image'
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

        self._spin_thread: Optional[threading.Thread] = None
        self._spinning = False

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
