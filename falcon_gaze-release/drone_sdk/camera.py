import json
import subprocess
import threading
import base64
from typing import Optional

import numpy as np

from .exceptions import CameraError


CAMERA_TOPIC = '/world/baylands_custom/model/x500_mono_cam_{}/link/mono_cam/base_link/sensor/camera_sensor/image'
GZ_CMD = 'gz'

_PIXEL_FORMAT_NAMES = {
    'RGB_INT8': 3, 'BGR_INT8': 3,
    'RGBA_INT8': 4, 'BGRA_INT8': 4,
    'L_INT8': 1, 'MONO8': 1,
}

_PIXEL_FORMAT_INTS = {
    3: 3, 8: 3,   # RGB_INT8 / BGR_INT8
    4: 4, 5: 4,   # RGBA_INT8 / BGRA_INT8
    1: 1,         # L_INT8
}

_RGB_NAMES = {'RGB_INT8', 'RGBA_INT8'}
_BGR_NAMES = {'BGR_INT8', 'BGRA_INT8'}


def _parse_image(json_msg: dict) -> Optional[np.ndarray]:
    try:
        width = json_msg['width']
        height = json_msg['height']
        data_b64 = json_msg.get('data', '')

        if not data_b64 or width == 0 or height == 0:
            return None

        raw = base64.b64decode(data_b64)

        pixel_fmt = json_msg.get('pixelFormatType', None)
        channels = 3

        if isinstance(pixel_fmt, str):
            channels = _PIXEL_FORMAT_NAMES.get(pixel_fmt, 3)
        elif isinstance(pixel_fmt, int):
            channels = _PIXEL_FORMAT_INTS.get(pixel_fmt, 3)

        expected = width * height * channels
        if len(raw) != expected:
            return None

        arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, channels)

        needs_rgb_to_bgr = (
            isinstance(pixel_fmt, str) and pixel_fmt in _RGB_NAMES
        ) or (
            isinstance(pixel_fmt, int) and pixel_fmt == 3  # RGB_INT8
        ) or (
            pixel_fmt is None and channels == 3
        )

        if needs_rgb_to_bgr and channels >= 3:
            import cv2
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

        return arr
    except Exception:
        return None


class CameraStream:

    def __init__(self, drone_id: int):
        self._drone_id = drone_id
        self._topic = CAMERA_TOPIC.format(drone_id)
        self._proc: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        try:
            self._proc = subprocess.Popen(
                [GZ_CMD, 'topic', '-e', '-t', self._topic, '--json-output'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                bufsize=1, universal_newlines=True,
            )
        except FileNotFoundError:
            self._running = False
            raise CameraError(f'{GZ_CMD} not found — is Gazebo installed?')

        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None

    def frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._latest

    def _reader(self) -> None:
        for line in self._proc.stdout:
            if not self._running:
                break
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
                img = _parse_image(msg)
                if img is not None:
                    with self._lock:
                        self._latest = img
            except json.JSONDecodeError:
                pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
