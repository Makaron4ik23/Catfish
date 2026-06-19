#!/usr/bin/env python3
"""
Offline dev calibration script.
Connects to Drone 0 (leader) and Drone 1 (follower) MAVSDK endpoints,
computes true physical 3D distance from position telemetry,
queries Follower 1 camera frames to detect target LED contour area,
and prints calibration statistics.
"""

import asyncio
import math
import sys
import os
import threading
import time
import rclpy
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from drone_sdk import Drone
from mavsdk import System
from examples.follower_mission import detect_led, TARGET_AREA

async def get_position(sys_instance):
    async for pos in sys_instance.telemetry.position_velocity_ned():
        return pos.position

async def run_calibration():
    rclpy.init()
    
    # Connect follower via SDK (starts bridges + camera)
    follower = Drone(drone_id=1)
    await follower.connect()
    follower.start_camera()
    
    # Connect to both drones via raw MAVSDK Systems to get positions
    sys_leader = System(port=50051)
    await sys_leader.connect(system_address="udpin://0.0.0.0:14540")
    
    sys_follower = System(port=50052)
    await sys_follower.connect(system_address="udpin://0.0.0.0:14541")
    
    print("Bridges active. Running calibration loop (Ctrl+C to stop)...")
    
    # Background ROS spin thread
    stop_event = threading.Event()
    def spin_thread():
        while not stop_event.is_set() and rclpy.ok():
            follower.spin()
            time.sleep(0.005)
            
    t = threading.Thread(target=spin_thread, daemon=True)
    t.start()
    
    try:
        while True:
            # Get telemetry positions
            pos_l = await get_position(sys_leader)
            pos_f = await get_position(sys_follower)
            
            # Compute physical NED distance
            dx = pos_l.north_m - pos_f.north_m
            dy = pos_l.east_m - pos_f.east_m
            dz = pos_l.down_m - pos_f.down_m
            real_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Get camera frame and detect LED area
            frame = follower.camera_frame()
            area = None
            if frame is not None:
                det = detect_led(frame)
                if det is not None:
                    _, _, _, _, area = det
                    
            if area is not None:
                # Target area corresponds to ~3.5 meters
                # We can estimate visual distance scaling as:
                # distance is inversely proportional to sqrt(area)
                # Let's say: est_dist = 3.5 * sqrt(TARGET_AREA / area)
                est_dist = 3.5 * math.sqrt(TARGET_AREA / area)
                error_pct = abs(est_dist - real_dist) / real_dist * 100.0
                flag = " [!]" if error_pct > 10.0 else ""
                print(f"Real Dist: {real_dist:.2f}m | Est Dist: {est_dist:.2f}m (Area: {area:.1f}) | Error: {error_pct:.1f}%{flag}")
            else:
                print(f"Real Dist: {real_dist:.2f}m | Est Dist: LOST (No LED)")
                
            await asyncio.sleep(0.5)
            
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        t.join(timeout=1.0)
        await follower.close()
        rclpy.shutdown()
        print("Calibration stopped.")

if __name__ == "__main__":
    asyncio.run(run_calibration())
