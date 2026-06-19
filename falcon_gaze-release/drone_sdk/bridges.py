import subprocess
import sys
from typing import List, Optional


CAMERA_TOPIC = '/world/baylands_custom/model/x500_mono_cam_{}/link/mono_cam/base_link/sensor/camera_sensor/image'
LED_TOPIC = '/model/x500_mono_cam_{}/led_cmd'


class BridgeManager:

    def __init__(self, drone_id: int):
        self._drone_id = drone_id
        self._camera_bridge: Optional[subprocess.Popen] = None
        self._led_bridge: Optional[subprocess.Popen] = None

    def start_camera_bridge(self) -> None:
        if self._camera_bridge is not None:
            return
        topic = CAMERA_TOPIC.format(self._drone_id)
        self._camera_bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_image', 'image_bridge', topic],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def start_led_bridge(self) -> None:
        if self._led_bridge is not None:
            return
        topic = LED_TOPIC.format(self._drone_id)
        bridge_arg = f"'{topic}@std_msgs/msg/String]gz.msgs.StringMsg'"
        self._led_bridge = subprocess.Popen(
            ['bash', '-c', f'ros2 run ros_gz_bridge parameter_bridge {bridge_arg}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def stop_all(self) -> None:
        for proc in [self._camera_bridge, self._led_bridge]:
            if proc is not None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self._camera_bridge = None
        self._led_bridge = None
