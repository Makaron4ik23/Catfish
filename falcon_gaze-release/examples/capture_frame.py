#!/usr/bin/env python3
import asyncio
import sys
import os
import cv2
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drone_sdk import Drone

async def main():
    rclpy.init()
    drone = Drone(drone_id=1)
    
    print("Connecting to drone 1...")
    try:
        await drone.connect()
        print("Connected!")
    except Exception as e:
        print(f"Connection failed: {e}")
        rclpy.shutdown()
        return

    print("Starting camera and waiting for frames...")
    drone.start_camera()
    
    frame_captured = False
    for i in range(200):
        drone.spin()
        frame = drone.camera_frame()
        if frame is not None:
            save_path = "/home/hacaton1/Desktop/Catfish/drone_1_capture.png"
            cv2.imwrite(save_path, frame)
            print(f"Frame captured and saved to {save_path}")
            frame_captured = True
            break
        await asyncio.sleep(0.05)
        
    if not frame_captured:
        print("Failed to capture frame from drone 1 camera.")
        
    drone.stop_camera()
    await drone.close()
    rclpy.shutdown()
    print("Done")

if __name__ == "__main__":
    asyncio.run(main())
