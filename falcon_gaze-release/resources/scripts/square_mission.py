import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

async def run():
    # Connect to the local SITL instance using the updated udpin:// protocol
    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected successfully!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off to 10 meters")
    await drone.action.set_takeoff_altitude(10.0)
    await drone.action.takeoff()
    
    # Wait for the drone to finish taking off and stabilize
    await asyncio.sleep(10)

    print("-- Setting initial setpoint to current position")
    # In NED coordinates: Z is negative for altitude (10m up = -10.0)
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))

    print("-- Starting Offboard Mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Fly a 10x10 meter square trajectory
    # Waypoint 1: Move 10m North
    print("-- Moving 10m North")
    await drone.offboard.set_position_ned(PositionNedYaw(10.0, 0.0, -10.0, 0.0))
    await asyncio.sleep(8)

    # Waypoint 2: Move 10m East (Forming a right angle)
    print("-- Moving 10m East")
    await drone.offboard.set_position_ned(PositionNedYaw(10.0, 10.0, -10.0, 90.0))
    await asyncio.sleep(8)

    # Waypoint 3: Move 10m South
    print("-- Moving 10m South")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 10.0, -10.0, 180.0))
    await asyncio.sleep(8)

    # Waypoint 4: Move 10m West (Back to home overhead)
    print("-- Returning to Home coordinate overhead")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 270.0))
    await asyncio.sleep(8)

    print("-- Stopping Offboard Mode")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
