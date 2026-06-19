import asyncio
from mavsdk import System

async def get_alt(port, name):
    drone = System(port=port)
    await drone.connect(system_address=f"udpin://0.0.0.0:{14540 + (port - 50051)}")
    print(f"Connecting to {name}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for pos in drone.telemetry.position():
        print(f"{name} absolute altitude: {pos.absolute_altitude_m:.2f} m, relative: {pos.relative_altitude_m:.2f} m")
        break

async def main():
    tasks = [
        get_alt(50051, "Leader (Drone 0)"),
        get_alt(50052, "Follower 1 (Drone 1)"),
        get_alt(50053, "Follower 2 (Drone 2)"),
        get_alt(50054, "Follower 3 (Drone 3)"),
    ]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
