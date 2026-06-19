import asyncio
from mavsdk import System

async def run():
    drone = System(port=50052)
    print("Connecting to drone 1...")
    await drone.connect(system_address="udpin://0.0.0.0:14541")
    print("Waiting for distance sensor telemetry...")
    async for dist in drone.telemetry.distance_sensor():
        print(f"Distance: min={dist.minimum_distance_m}, max={dist.maximum_distance_m}, cur={dist.current_distance_m}, orientation={dist.orientation}")

if __name__ == "__main__":
    asyncio.run(run())
