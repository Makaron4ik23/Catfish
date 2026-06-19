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

    try:
        await drone.param.set_param_int("COM_LOW_BAT_ACT", 0)
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
    await asyncio.sleep(5) 

    print(f"-- Taking off to {takeoff_alt_m:.2f}m above platform")
    await drone.action.set_takeoff_altitude(takeoff_alt_m)
    await drone.action.takeoff()
    await asyncio.sleep(10) # Let it reach a completely stable hover

# =========================================================================
    # DYNAMIC TELEMETRY CAPTURE (FIXED)
    # Capture the exact heading and altitude the drone holds right now in hover.
    # =========================================================================
    print("-- Capturing baseline orientation...")
    
    # 1. Takeoff target altitude is takeoff_alt_m
    hover_absolute_alt = takeoff_alt_m
        
    # 2. Grab the current absolute heading (Yaw reference) by averaging multiple readings to bypass MAVSDK's stale buffer.
    hover_yaw_deg = 0.0
    count = 0
    async for heading in drone.telemetry.heading():
        hover_yaw_deg += heading.heading_deg
        count += 1
        if count >= 10:
            break
    hover_yaw_deg /= count
        
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
        return

    # ==========================
    # CORE CAMERA TRAINING PATTERN
    # ==========================

    # --- Phase 1: Move on path 2 meters ---
    print("-> Moving forward 2 meters on path (LED ON -> FOLLOW)")
    drone_leds.led_on()
    await drone.offboard.set_position_ned(get_ned_position(2.0, 0.0, 0.0))
    await asyncio.sleep(6)

    # --- Sub-routine 1: Cross & Altitude Pattern ---
    print("-> Pattern 1: Perpendicular left 2m")
    await drone.offboard.set_position_ned(get_ned_position(2.0, 2.0, 0.0))
    await asyncio.sleep(5)

    print("-> Pattern 1: Perpendicular right 2m")
    await drone.offboard.set_position_ned(get_ned_position(2.0, -2.0, 0.0))
    await asyncio.sleep(7)

    print("-> Pattern 1: Return to center line")
    await drone.offboard.set_position_ned(get_ned_position(2.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("-> Pattern 1: Climb up +2 meters")
    await drone.offboard.set_position_ned(get_ned_position(2.0, 0.0, 2.0))
    await asyncio.sleep(5)

    print("-> Pattern 1: Climb down -2 meters")
    await drone.offboard.set_position_ned(get_ned_position(2.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("-> ** HOLD Phase ** (Manual 1Hz BLINK -> HOLD)")
    for _ in range(10):
        drone_leds.led_on()
        await asyncio.sleep(0.5)
        drone_leds.led_off()
        await asyncio.sleep(0.5)

    # --- Phase 2: Progress down path to +5 meters total ---
    print("-> Moving forward an additional 3 meters (Total: 5m down path) (LED ON -> FOLLOW)")
    drone_leds.led_on()
    await drone.offboard.set_position_ned(get_ned_position(5.0, 0.0, 0.0))
    await asyncio.sleep(6)

    # --- Sub-routine 2: Repeat Pattern ---
    print("-> Pattern 2: Perpendicular left 2m")
    await drone.offboard.set_position_ned(get_ned_position(5.0, 2.0, 0.0))
    await asyncio.sleep(5)

    print("-> Pattern 2: Perpendicular right 2m")
    await drone.offboard.set_position_ned(get_ned_position(5.0, -2.0, 0.0))
    await asyncio.sleep(7)

    print("-> Pattern 2: Return to center line")
    await drone.offboard.set_position_ned(get_ned_position(5.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("-> Pattern 2: Climb up +2 meters")
    await drone.offboard.set_position_ned(get_ned_position(5.0, 0.0, 2.0))
    await asyncio.sleep(5)

    print("-> Pattern 2: Climb down -2 meters")
    await drone.offboard.set_position_ned(get_ned_position(5.0, 0.0, 0.0))
    await asyncio.sleep(5)

    # --- Phase 3: Return & Land ---
    print("-> Returning backwards to start position (0,0)")
    await drone.offboard.set_position_ned(get_ned_position(0.0, 0.0, 0.0))
    await asyncio.sleep(8)

    print("-- Stopping Offboard Mode to allow landing action")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed: {error._result.result}")

    print("-- Landing (LED OFF -> FINISH/LOST)")
    drone_leds.led_off()
    await drone.action.land()
    
if __name__ == "__main__":
    asyncio.run(run())