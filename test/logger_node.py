#!/usr/bin/env python3
"""
Logs drone positions + LED states to JSON.

Positions: read directly from Gazebo via:
    gz topic -e -t /world/baylands_custom/pose/info
    Parses protobuf text format for x500_mono_cam_{id} model poses.
    ENU → NED: north=pos.y, east=pos.x, down=-pos.z

LED states: ROS2 subscriber to /model/x500_mono_cam_{id}/led_cmd
            (published by Drone SDK's ROS2 node).

Usage:
  python3 test/logger_node.py [--drones 4]
"""

import argparse
import json
import os
import re
import subprocess
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

LOG_DIR = os.path.join(os.path.dirname(__file__), 'logs')
DRONE_MODEL_PREFIX = 'x500_mono_cam_'
POSE_TOPIC = '/world/baylands_custom/pose/info'

_RE_NAME = re.compile(r'name:\s*"([^"]+)"')
_RE_POS_X = re.compile(r'x:\s*(-?[\d.]+(?:e[+-]?\d+)?)')
_RE_POS_Y = re.compile(r'y:\s*(-?[\d.]+(?:e[+-]?\d+)?)')
_RE_POS_Z = re.compile(r'z:\s*(-?[\d.]+(?:e[+-]?\d+)?)')


def parse_pose_v(text: str) -> dict:
    """Parse protobuf text format of gz.msgs.Pose_V.

    Returns {model_name: (north, east, down), ...}
    """
    poses = {}
    blocks = text.split('pose {')
    for block in blocks[1:]:
        m = _RE_NAME.search(block)
        if not m:
            continue
        name = m.group(1)
        xs = _RE_POS_X.search(block)
        ys = _RE_POS_Y.search(block)
        zs = _RE_POS_Z.search(block)
        if xs and ys and zs:
            gz_x = float(xs.group(1))
            gz_y = float(ys.group(1))
            gz_z = float(zs.group(1))
            poses[name] = (gz_y, gz_x, -gz_z)
    return poses


class GzPoseReader:

    def __init__(self, topic: str, drone_ids: list, callback):
        self._topic = topic
        self._drone_ids = drone_ids
        self._callback = callback
        self._process: subprocess.Popen = None
        self._thread: threading.Thread = None
        self._running = False

    def start(self):
        self._running = True
        self._process = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', self._topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._process:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._process.kill()

    def _read_loop(self):
        buffer = ''
        for line in self._process.stdout:
            if not self._running:
                break
            buffer += line
            if line.strip() == '}' and 'pose {' in buffer:
                poses = parse_pose_v(buffer)
                buffer = ''
                now = time.time()
                for did in self._drone_ids:
                    model = f'{DRONE_MODEL_PREFIX}{did}'
                    if model in poses:
                        n, e, d = poses[model]
                        self._callback(did, now, n, e, d)


class DroneLoggerNode(Node):

    # Spawn ENU: (127.0, 52.67, 1.4) → NED: north=52.67, east=127.0
    # Start platform is 6×6 m, centered at (128.2, 53.5), nearest edge
    # ~1.5 m from spawn. Start logging once drone 0 exceeds this radius.
    _SPAWN_N = 52.67
    _SPAWN_E = 127.0
    _PLATFORM_RADIUS_M = 1.5

    def __init__(self, num_drones: int):
        super().__init__('drone_logger')
        self._num_drones = num_drones
        self._samples: list = []
        self._led_states: dict = {i: '' for i in range(num_drones)}
        self._sample_count = 0
        self._lock = threading.Lock()
        self._started = False

        for i in range(num_drones):
            led_topic = f'/model/x500_mono_cam_{i}/led_cmd'
            self.create_subscription(
                String, led_topic,
                lambda msg, did=i: self._led_cb(msg, did), 10)

        self._reader = GzPoseReader(
            POSE_TOPIC, list(range(num_drones)), self._pose_sample)
        self._reader.start()

        start = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        self.get_logger().info(
            f'Logger started at {start} — {num_drones} drones, '
            f'reading {POSE_TOPIC} via gz topic')
        print(f'Awaiting drone 0 departure from spawn ({self._SPAWN_N:.1f} N, {self._SPAWN_E:.1f} E)…', flush=True)

    def _pose_sample(self, drone_id: int, t: float, n: float, e: float, d: float):
        with self._lock:
            if not self._started:
                if drone_id == 0:
                    dx = n - self._SPAWN_N
                    dy = e - self._SPAWN_E
                    dist = (dx*dx + dy*dy) ** 0.5
                    if dist > self._PLATFORM_RADIUS_M:
                        self._started = True
                        self.get_logger().info(
                            f'Drone 0 left platform (dist={dist:.2f}m), logging started')
                        print(f'\n=== LOGGING STARTED (drone 0 {dist:.1f}m from spawn) ===\n', flush=True)
                if not self._started:
                    return
            self._samples.append({
                't': t,
                'id': drone_id,
                'n': n,
                'e': e,
                'd': d,
                'led': self._led_states.get(drone_id, ''),
            })
            self._sample_count += 1

    def _led_cb(self, msg: String, drone_id: int):
        self._led_states[drone_id] = msg.data

    def _save(self):
        with self._lock:
            if not self._samples:
                self.get_logger().info('No samples to save yet')
                return
            samples = list(self._samples)
        grouped = {}
        for s in samples:
            name = f'drone_{s["id"]}'
            grouped.setdefault(name, []).append({
                't': s['t'],
                'n': s['n'],
                'e': s['e'],
                'd': s['d'],
                'led': s['led'],
            })
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        path = os.path.join(LOG_DIR, f'flight_{ts}.json')
        data = {
            'meta': {
                'start_time': ts,
                'num_drones': self._num_drones,
                'num_samples': self._sample_count,
            },
            'data': grouped,
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'Saved {self._sample_count} samples to {path}')

    def save_on_shutdown(self):
        try:
            self._save()
        except Exception:
            pass

    def cleanup(self):
        self._reader.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--drones', type=int, default=4)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = DroneLoggerNode(args.drones)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.cleanup()
        node.save_on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
