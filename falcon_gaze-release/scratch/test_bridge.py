#!/usr/bin/env python3
import subprocess
import time
import sys

topic = '/world/baylands_custom/model/x500_mono_cam_1/link/mono_cam/base_link/sensor/camera_sensor/image'
cmd = ['ros2', 'run', 'ros_gz_image', 'image_bridge', topic]

print(f"Running command: {' '.join(cmd)}")
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

time.sleep(3.0)
ret = proc.poll()
print(f"Return code: {ret}")

stdout, stderr = proc.communicate(timeout=1.0)
print("STDOUT:")
print(stdout)
print("STDERR:")
print(stderr)
