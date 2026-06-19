import asyncio
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

# --- Configuration Constants ---
TAKEOFF_ALT_M = 1.0     # Take off exactly 1 meter above the platform

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(5) 

    print(f"-- Taking off to {TAKEOFF_ALT_M}m above platform")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.takeoff()
    await asyncio.sleep(10) # Let it reach a completely stable hover

# =========================================================================
    # DYNAMIC TELEMETRY CAPTURE (FIXED)
    # Capture the exact heading and altitude the drone holds right now in hover.
    # =========================================================================
    print("-- Capturing baseline orientation...")
    
    # 1. Grab the current absolute altitude (Z-axis reference)
    async for position in drone.telemetry.position():
        hover_absolute_alt = position.relative_altitude_m
        break # Stop listening after getting the first stable reading
        
    # 2. Grab the current absolute heading (Yaw reference)
    async for heading in drone.telemetry.heading():
        hover_yaw_deg = heading.heading_deg  # <-- Fixed attribute name here!
        break # Stop listening after getting the first stable reading
        
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
    print("-> Moving forward 2 meters on path")
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

    # --- Phase 2: Progress down path to +5 meters total ---
    print("-> Moving forward an additional 3 meters (Total: 5m down path)")
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

    print("-- Landing")
    await drone.action.land()
    
if __name__ == "__main__":
    asyncio.run(run())