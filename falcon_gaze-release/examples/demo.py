#!/usr/bin/env python3
"""
Standalone example demonstrating the drone_sdk wrapper (ROS2 backend).

Inlines ROS2 spin so you can retrieve and process frames directly.

Run:
  source /opt/ros/humble/setup.bash
  python3 examples/demo.py
"""

import asyncio
import sys
import os
import signal
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import rclpy
from drone_sdk import Drone


async def show_position(drone: Drone, label: str):
    pos = await drone.position_ned()
    print(f'  {label}: N={pos.north_m:.1f} E={pos.east_m:.1f} D={pos.down_m:.1f}')


async def led_pattern(drone: Drone, shutdown: asyncio.Event):
    masks = ['1000', '0100', '0010', '0001']
    while not shutdown.is_set():
        for mask in masks:
            if shutdown.is_set():
                return
            drone.set_leds(mask)
            await asyncio.sleep(1.0)
        if shutdown.is_set():
            return
        drone.led_blink()
        await asyncio.sleep(2.0)


def camera_loop(drone: Drone, shutdown: threading.Event):
    """Run in a dedicated thread: spin + retrieve + display."""
    drone.start_camera()
    win = f'Drone {drone.drone_id} Camera'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    sized = False
    while not shutdown.is_set() and rclpy.ok():
        drone.spin()
        frame = drone.camera_frame()
        if frame is not None:
            if not sized:
                h, w = frame.shape[:2]
                cv2.resizeWindow(win, w, h)
                sized = True
            cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
    cv2.destroyWindow(win)
    drone.stop_camera()


async def main():
    rclpy.init()
    drone = Drone(drone_id=0)
    shutdown = threading.Event()
    shutdown_async = asyncio.Event()

    def on_sig(*_):
        shutdown.set()
        shutdown_async.set()
    signal.signal(signal.SIGINT, on_sig)

    print('Connecting to drone 0...')
    try:
        await drone.connect()
    except Exception as e:
        print(f'Connection failed: {e}')
        rclpy.shutdown()
        return

    print('Connected!')
    await show_position(drone, 'Initial')

    cam_thread = threading.Thread(
        target=camera_loop, args=(drone, shutdown), daemon=True
    )
    cam_thread.start()

    led_task = asyncio.create_task(led_pattern(drone, shutdown_async))

    print('Arming...')
    await drone.arm()
    print('Armed')    

    print('Takeoff to 10m...')
    await drone.takeoff(altitude_m=10.0)
    await asyncio.sleep(12)
    await show_position(drone, 'After takeoff')
    print('Takeoff complete')

    print('Starting offboard control...')
    await drone.start_offboard()
    await show_position(drone, 'Offboard started')
    print('Offboard active')

    print('Moving 10m forward...')
    await drone.go_to(10.0, 0.0, 0.0, body_frame=True)
    await asyncio.sleep(6)
    await show_position(drone, 'After move 1')

    print('Moving 10m right...')
    await drone.go_to(0.0, 10.0, 0.0, body_frame=True)
    await asyncio.sleep(6)
    await show_position(drone, 'After move 2')

    print('Returning to origin...')
    await drone.go_to(-10.0, -10.0, 0.0, body_frame=True)
    await asyncio.sleep(6)
    await show_position(drone, 'After return')

    print('Stopping offboard...')
    await drone.stop_offboard()

    print('Landing...')
    await drone.land()
    await asyncio.sleep(20)
    await show_position(drone, 'After land')

    print('Disarming...')
    await drone.disarm()

    shutdown_async.set()
    led_task.cancel()
    try:
        await led_task
    except (asyncio.CancelledError, Exception):
        pass

    shutdown.set()
    cam_thread.join(timeout=5)
    await drone.close()
    rclpy.shutdown()
    print('Done')


if __name__ == '__main__':
    asyncio.run(main())
