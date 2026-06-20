#!/usr/bin/env python3
"""Helper drones (ids 1-3). Same world points as leader — position
feedback ensures real physical separation."""

import asyncio
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drone_sdk import Drone, MAVSDKError

STAGGER = 2.0
ARRIVAL_THRESHOLD = 2.0
LEG_TIMEOUT = 30.0
YAW = 3.7346

SPAWN_ENU = {
    0: (127.0, 52.67, 1.4), 1: (129.92, 52.852, 1.4),
    2: (129.08, 54.095, 1.4), 3: (128.24, 55.339, 1.4),
}

BODY_PATH = [
    (10, 0), (20, 0), (30, 0), (40, 0),
    (40, 10), (40, 20),
    (30, 20), (20, 20), (10, 20), (0, 20),
    (0, 10), (0, 0),
]
ALTITUDE_M = 5.0


def body_to_enu(fwd: float, right: float):
    sy = math.sin(YAW)
    cy = math.cos(YAW)
    return (fwd * sy + right * cy,
            fwd * cy - right * sy)


def world_enu_waypoints():
    sx, sy, sz = SPAWN_ENU[0]
    alt = sz + ALTITUDE_M
    return [(sx + ex, sy + ny, alt) for ex, ny in (body_to_enu(f, r) for f, r in BODY_PATH)]


def ned_for_drone(drone_id, world_enu_wpts):
    sn, se, sd = SPAWN_ENU[drone_id]
    return [(wy - se, wx - sn, -(wz - sd)) for wx, wy, wz in world_enu_wpts]


async def led_pattern(drones: list, shutdown: asyncio.Event):
    masks = ['1000', '0100', '0010', '0001']
    idx = 0
    while not shutdown.is_set():
        mask = masks[idx % len(masks)]
        for d in drones:
            d.set_leds(mask)
        idx += 1
        await asyncio.sleep(1.0)
        if shutdown.is_set():
            return
        for d in drones:
            d.led_blink()
        await asyncio.sleep(2.0)


async def await_arrival(drone, target_n, target_e, target_d, timeout=LEG_TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            pos = await drone.position_ned()
        except Exception:
            await asyncio.sleep(0.25)
            continue
        dist = math.sqrt(
            (pos.north_m - target_n) ** 2 +
            (pos.east_m - target_e) ** 2 +
            (pos.down_m - target_d) ** 2
        )
        if dist < ARRIVAL_THRESHOLD:
            return True
        await asyncio.sleep(0.25)
    return False


async def mission(drone, drone_id, stop_event, shutdown, waypoints):
    offset = drone_id * STAGGER
    offboard_active = False
    try:
        print(f'  Drone {drone_id}: arm')
        await drone.arm()
        print(f'  Drone {drone_id}: takeoff')
        await drone.takeoff(altitude_m=10.0)
        await asyncio.sleep(14)

        for attempt in range(3):
            try:
                print(f'  Drone {drone_id}: start offboard (attempt {attempt+1})')
                await drone.start_offboard()
                offboard_active = True
                break
            except MAVSDKError as e:
                msg = str(e)
                if attempt < 2 and ('COMMAND_DENIED' in msg or 'NO_SETPOINT_SET' in msg):
                    print(f'  Drone {drone_id}: offboard {msg.split(":")[-1].strip()}, retry in 2s')
                    await asyncio.sleep(2)
                    continue
                raise

        for i, (n, e, d) in enumerate(waypoints):
            if stop_event.is_set() or shutdown.is_set():
                break
            if i == 0 and offset > 0:
                print(f'  Drone {drone_id}: initial stagger {offset:.0f}s')
                await asyncio.sleep(offset)
            print(f'  Drone {drone_id}: leg {i+1} → NED ({n:.1f}, {e:.1f}, {d:.1f})')
            await drone.go_to(n, e, d)
            arrived = await await_arrival(drone, n, e, d)
            if not arrived:
                print(f'  Drone {drone_id}: leg {i+1} timed out')

        if not (stop_event.is_set() or shutdown.is_set()):
            print(f'  Drone {drone_id}: return to own spawn')
            await drone.go_to(0, 0, -5)
            await await_arrival(drone, 0, 0, -5)

    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f'  Drone {drone_id}: flight error: {e}')

    if stop_event.is_set() or shutdown.is_set():
        return

    try:
        if offboard_active:
            print(f'  Drone {drone_id}: stop offboard')
            await drone.stop_offboard()
        print(f'  Drone {drone_id}: land')
        await drone.land()
        print(f'  Drone {drone_id}: wait 20s before disarm')
        await asyncio.sleep(20)
        await drone.disarm()
        print(f'  Drone {drone_id}: done')
    except Exception as e:
        print(f'  Drone {drone_id}: land error: {e}')


async def main():
    stop_event = asyncio.Event()
    shutdown_async = asyncio.Event()

    def on_sig(*_):
        stop_event.set()
        shutdown_async.set()
    signal.signal(signal.SIGINT, on_sig)

    import rclpy
    rclpy.init()

    drones = [Drone(drone_id=i) for i in [1, 2, 3]]

    print('Connecting helper drones...')
    for d in drones:
        try:
            await d.connect()
            print(f'  Drone {d.drone_id}: connected')
        except Exception as e:
            print(f'  Drone {d.drone_id}: connect failed: {e}')
            stop_event.set()

    wpts_all = world_enu_waypoints()
    drone_wpts = {did: ned_for_drone(did, wpts_all) for did in [1, 2, 3]}

    led_task = asyncio.create_task(led_pattern(drones, shutdown_async))
    missions = [asyncio.create_task(
        mission(d, d.drone_id, stop_event, shutdown_async, drone_wpts[d.drone_id]))
        for d in drones]

    await asyncio.gather(*missions)

    shutdown_async.set()
    led_task.cancel()
    try:
        await led_task
    except asyncio.CancelledError:
        pass

    for d in drones:
        await d.close()
    rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
