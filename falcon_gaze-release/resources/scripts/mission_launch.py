import asyncio
import math
import os
import sys
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
import rclpy
from drone_sdk import Drone

# --- Configuration Constants ---
TARGET_ABS_ALT = 42.0   # Target absolute altitude (MSL) for the entire swarm

async def run():
    rclpy.init()
    drone_leds = Drone(drone_id=0)
    drone_leds.led_on()

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    # ── High-Speed Telemetry background tracking ─────────────────
    curr_n, curr_e, curr_d = 0.0, 0.0, 0.0
    curr_vn, curr_ve, curr_vd = 0.0, 0.0, 0.0
    curr_yaw = 0.0
    curr_abs_alt = 0.0

    async def track_telemetry_loop():
        nonlocal curr_n, curr_e, curr_d, curr_vn, curr_ve, curr_vd, curr_yaw, curr_abs_alt
        async def fetch_heading():
            nonlocal curr_yaw
            try:
                async for hdg in drone.telemetry.heading():
                    curr_yaw = hdg.heading_deg
            except asyncio.CancelledError:
                pass
        async def fetch_abs_alt():
            nonlocal curr_abs_alt
            try:
                async for pos in drone.telemetry.position():
                    curr_abs_alt = pos.absolute_altitude_m
            except asyncio.CancelledError:
                pass
        async def fetch_pos_vel():
            nonlocal curr_n, curr_e, curr_d, curr_vn, curr_ve, curr_vd
            try:
                async for pv in drone.telemetry.position_velocity_ned():
                    curr_n = pv.position.north_m
                    curr_e = pv.position.east_m
                    curr_d = pv.position.down_m
                    curr_vn = pv.velocity.north_m_s
                    curr_ve = pv.velocity.east_m_s
                    curr_vd = pv.velocity.down_m_s
            except asyncio.CancelledError:
                pass
        async def publish_loop():
            try:
                while True:
                    drone_leds.publish_telemetry(curr_n, curr_e, curr_d, curr_vn, curr_ve, curr_vd, curr_yaw, curr_abs_alt)
                    await asyncio.sleep(0.05)
            except asyncio.CancelledError:
                pass

        tasks = [
            asyncio.create_task(fetch_heading()),
            asyncio.create_task(fetch_abs_alt()),
            asyncio.create_task(fetch_pos_vel()),
            asyncio.create_task(publish_loop())
        ]
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            pass
        finally:
            for t in tasks:
                t.cancel()

    telemetry_task = asyncio.create_task(track_telemetry_loop())

    # Disable ALL battery failsafes for SITL simulation
    for name, val in [("COM_LOW_BAT_ACT", 0)]:
        try:
            await drone.param.set_param_int(name, val)
        except Exception:
            pass
    for name, val in [("BAT_LOW_THR", 0.02), ("BAT_CRIT_THR", 0.01), ("BAT_EMERGEN_THR", 0.005)]:
        try:
            await drone.param.set_param_float(name, val)
        except Exception:
            pass

    # Get ground absolute altitude to compute relative takeoff altitude
    ground_abs_alt = None
    async for pos in drone.telemetry.position():
        ground_abs_alt = pos.absolute_altitude_m
        break
    takeoff_alt_m = TARGET_ABS_ALT - ground_abs_alt
    print(f"Ground absolute altitude: {ground_abs_alt:.2f}m. Takeoff relative altitude: {takeoff_alt_m:.2f}m")

    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(3)  # fast arming

    print(f"-- Taking off to {takeoff_alt_m:.2f}m above platform")
    await drone.action.set_takeoff_altitude(takeoff_alt_m)
    await drone.action.takeoff()
    await asyncio.sleep(4) # fast takeoff sleep

# =========================================================================
    # DYNAMIC TELEMETRY CAPTURE (FIXED)
    # Capture the exact heading and altitude the drone holds right now in hover.
    # =========================================================================
    print("-- Capturing baseline orientation...")
    
    # 1. Takeoff target altitude is takeoff_alt_m
    hover_absolute_alt = takeoff_alt_m
        
    # 2. Grab the current absolute heading (Yaw reference) from our running background tracker.
    while curr_yaw == 0.0:
        await asyncio.sleep(0.1)
    hover_yaw_deg = curr_yaw
        
    hover_yaw_rad = math.radians(hover_yaw_deg)
    print(f"Captured Base - Altitude: {hover_absolute_alt:.2f}m, Heading: {hover_yaw_deg:.2f}°")
    # Helper function using our freshly captured real-time bases
    def get_ned_position(forward, left, altitude_change):
        # Translate local forward/left steps based on the drone's true active heading
        north = (forward * math.cos(hover_yaw_rad)) + (left * math.sin(hover_yaw_rad))
        east = (forward * math.sin(hover_yaw_rad)) - (left * math.cos(hover_yaw_rad))
        
        # Target altitude is relative to our exact stable hover altitude
        target_alt = hover_absolute_alt + altitude_change
        down = -target_alt
        
        return PositionNedYaw(north, east, down, hover_yaw_deg)

    print("-- Preparing Offboard stream")
    # Feed an initial setpoint at its current exact location
    await drone.offboard.set_position_ned(get_ned_position(0.0, 0.0, 0.0))

    print("-- Starting Offboard Mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        telemetry_task.cancel()
        return

    # ==========================
    # OFFICIAL MISSION PATH
    # ==========================

    # --- Start -> Checkpoint 1 (10m forward) ---
    print("-> Moving to Checkpoint 1 (10m forward). LED ON -> FOLLOW")
    drone_leds.led_on()
    await drone.offboard.set_position_ned(get_ned_position(10.0, 0.0, 0.0))
    await asyncio.sleep(15)  # Time to travel 10 meters

    # --- Checkpoint 1 HOLD ---
    print("-> ** Checkpoint 1 HOLD ** (Manual 5Hz BLINK -> HOLD)")
    for _ in range(25):  # 5 seconds hold
        drone_leds.led_on()
        await asyncio.sleep(0.1)
        drone_leds.led_off()
        await asyncio.sleep(0.1)

    # --- Checkpoint 1 -> Checkpoint 2 (20m forward, 5m right) ---
    print("-> Moving to Checkpoint 2 (20m forward, 5m right). LED ON -> FOLLOW")
    drone_leds.led_on()
    await drone.offboard.set_position_ned(get_ned_position(20.0, 5.0, 0.0))
    await asyncio.sleep(15)

    # --- Checkpoint 2 HOLD ---
    print("-> ** Checkpoint 2 HOLD ** (Manual 5Hz BLINK -> HOLD)")
    for _ in range(25):
        drone_leds.led_on()
        await asyncio.sleep(0.1)
        drone_leds.led_off()
        await asyncio.sleep(0.1)

    # --- Checkpoint 2 -> Target B (30m forward, 5m right) ---
    print("-> Moving to Target B (30m forward, 5m right). LED ON -> FOLLOW")
    drone_leds.led_on()
    await drone.offboard.set_position_ned(get_ned_position(30.0, 5.0, 0.0))
    await asyncio.sleep(15)

    print("-- Stopping Offboard Mode to allow landing action")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed: {error._result.result}")

    print("-- Landing (LED OFF -> FINISH/LOST)")
    drone_leds.led_off()
    await drone.action.land()
    telemetry_task.cancel()
    
if __name__ == "__main__":
    asyncio.run(run())