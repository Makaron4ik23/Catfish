#!/usr/bin/env python3
"""
SDK-based swarm demo: 4 drones fly forward in formation, land after 40s.

Camera windows stay open after landing. Press Ctrl+C or 'q' to quit.

Run:
  source /opt/ros/humble/setup.bash
  python3 examples/swarm.py
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

SWARM_SIZE = 4
ALTITUDE = 10.0
SPEED = 2.0
FLIGHT_DURATION = 40


def camera_loop(drones: list, shutdown: threading.Event):
    windows = {}
    for d in drones:
        d.start_camera()
        win = f'Drone {d.drone_id}'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        windows[d.drone_id] = {'name': win, 'sized': False}

    while not shutdown.is_set() and rclpy.ok():
        for d in drones:
            d.spin()
            frame = d.camera_frame()
            if frame is not None:
                entry = windows[d.drone_id]
                if not entry['sized']:
                    h, w = frame.shape[:2]
                    cv2.resizeWindow(entry['name'], w, h)
                    entry['sized'] = True
                cv2.imshow(entry['name'], frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    for entry in windows.values():
        cv2.destroyWindow(entry['name'])
    for d in drones:
        d.stop_camera()


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


async def drone_mission(drone: Drone, drone_id: int,
                        stop_event: asyncio.Event, shutdown: asyncio.Event):
    try:
        print(f'Drone {drone_id}: arming')
        await drone.arm()
        print(f'Drone {drone_id}: takeoff')
        await drone.takeoff(altitude_m=ALTITUDE)
        await asyncio.sleep(14)
        print(f'Drone {drone_id}: start offboard')
        await drone.start_offboard()
        print(f'Drone {drone_id}: flying forward')

        while not stop_event.is_set() and not shutdown.is_set():
            await drone.move(1.0, 0.0, 0.0, speed_m_s=SPEED)
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f'Drone {drone_id}: error: {e}')

    # Stop, land, disarm
    try:
        print(f'Drone {drone_id}: stopping')
        await drone.move(0.0, 0.0, 0.0, speed_m_s=0.0)
        await asyncio.sleep(0.5)
        print(f'Drone {drone_id}: landing')
        await drone.stop_offboard()
        await drone.land()
        await asyncio.sleep(20)
        print(f'Drone {drone_id}: disarming')
        await drone.disarm()
        print(f'Drone {drone_id}: landed')
    except Exception as e:
        print(f'Drone {drone_id}: land error: {e}')


async def main():
    rclpy.init()
    shutdown_event = threading.Event()
    shutdown_async = asyncio.Event()
    stop_event = asyncio.Event()

    def on_sig(*_):
        shutdown_event.set()
        shutdown_async.set()
        stop_event.set()
    signal.signal(signal.SIGINT, on_sig)

    drones = [Drone(drone_id=i) for i in range(SWARM_SIZE)]

    print('Connecting all drones...')
    for d in drones:
        try:
            await d.connect()
            print(f'  Drone {d.drone_id}: connected')
        except Exception as e:
            print(f'  Drone {d.drone_id}: connect failed: {e}')
            stop_event.set()

    cam_thread = threading.Thread(
        target=camera_loop, args=(drones, shutdown_event), daemon=True
    )
    cam_thread.start()
    await asyncio.sleep(0.5)

    led_task = asyncio.create_task(led_pattern(drones[0], shutdown_async))

    missions = []
    for i, d in enumerate(drones):
        missions.append(asyncio.create_task(
            drone_mission(d, i, stop_event, shutdown_async)))

    print(f'Mission started — drones will land after {FLIGHT_DURATION}s')
    await asyncio.sleep(FLIGHT_DURATION)
    print('Mission time complete — stopping drones')
    stop_event.set()

    await asyncio.gather(*missions, return_exceptions=True)
    led_task.cancel()
    print('Swarm complete')


if __name__ == '__main__':
    asyncio.run(main())
