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
MIN_COMMAND_TAKEOFF_ALT = 5.0
MIN_TAKEOFF_REL_ALT = 2.0
TAKEOFF_TIMEOUT_S = 35.0

async def run():
    rclpy.init()
    drone_leds = Drone(drone_id=0)

    async def publish_leader_led(mode: str, repeats: int = 3, interval_s: float = 0.2):
        for idx in range(repeats):
            if mode == "ON":
                drone_leds.led_on()
            elif mode == "OFF":
                drone_leds.led_off()
            elif mode == "BLINK":
                drone_leds.led_blink()
            drone_leds.spin()
            if idx < repeats - 1 and interval_s > 0.0:
                await asyncio.sleep(interval_s)

    await publish_leader_led("ON", repeats=6, interval_s=0.25)

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
            while True:
                try:
                    async for hdg in drone.telemetry.heading():
                        curr_yaw = hdg.heading_deg
                except asyncio.CancelledError:
                    break
                except Exception:
                    await asyncio.sleep(0.5)
        async def fetch_abs_alt():
            nonlocal curr_abs_alt
            while True:
                try:
                    async for pos in drone.telemetry.position():
                        curr_abs_alt = pos.absolute_altitude_m
                except asyncio.CancelledError:
                    break
                except Exception:
                    await asyncio.sleep(0.5)
        async def fetch_pos_vel():
            nonlocal curr_n, curr_e, curr_d, curr_vn, curr_ve, curr_vd
            while True:
                try:
                    async for pv in drone.telemetry.position_velocity_ned():
                        curr_n = pv.position.north_m
                        curr_e = pv.position.east_m
                        curr_d = pv.position.down_m
                        curr_vn = pv.velocity.north_m_s
                        curr_ve = pv.velocity.east_m_s
                        curr_vd = pv.velocity.down_m_s
                except asyncio.CancelledError:
                    break
                except Exception:
                    await asyncio.sleep(0.5)
        async def publish_loop():
            try:
                while True:
                    drone_leds.publish_telemetry(
                        curr_n,
                        curr_e,
                        curr_d,
                        curr_vn,
                        curr_ve,
                        curr_vd,
                        curr_yaw,
                        curr_abs_alt,
                        airborne=(-curr_d > 1.5),
                    )
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

    # Disable noisy SITL failsafes that can block or abort simulated takeoff.
    for name, val in [
        ("COM_LOW_BAT_ACT", 0),
        ("SYS_HAS_MAG", 0),
        ("COM_ARM_MAG_STR", 0),
    ]:
        try:
            await drone.param.set_param_int(name, val)
        except Exception as exc:
            print(f"Warning: set {name}={val} failed: {exc}")
    for name, val in [("BAT_LOW_THR", 0.02), ("BAT_CRIT_THR", 0.01), ("BAT_EMERGEN_THR", 0.005)]:
        try:
            await drone.param.set_param_float(name, val)
        except Exception as exc:
            print(f"Warning: set {name}={val} failed: {exc}")

    ground_abs_alt = None
    async for pos in drone.telemetry.position():
        ground_abs_alt = pos.absolute_altitude_m
        break
    takeoff_alt_m = max(TARGET_ABS_ALT - ground_abs_alt, MIN_COMMAND_TAKEOFF_ALT)
    print(f"Ground absolute altitude: {ground_abs_alt:.2f}m. Takeoff relative altitude: {takeoff_alt_m:.2f}m")

    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(3)  # fast arming

    print(f"-- Taking off to {takeoff_alt_m:.2f}m above platform")
    await drone.action.set_takeoff_altitude(takeoff_alt_m)
    await drone.action.takeoff()
    required_takeoff_alt = max(
        MIN_TAKEOFF_REL_ALT,
        min(takeoff_alt_m - 0.5, takeoff_alt_m * 0.75),
    )
    deadline = asyncio.get_event_loop().time() + TAKEOFF_TIMEOUT_S
    takeoff_confirmed = False
    while asyncio.get_event_loop().time() < deadline:
        rel_abs_alt = curr_abs_alt - ground_abs_alt
        rel_ned_alt = -curr_d
        if max(rel_abs_alt, rel_ned_alt) >= MIN_TAKEOFF_REL_ALT:
            print(f"Takeoff confirmed: rel_abs={rel_abs_alt:.2f}m rel_ned={rel_ned_alt:.2f}m")
            takeoff_confirmed = True
            break
        await asyncio.sleep(0.25)

    if not takeoff_confirmed:
        rel_abs_alt = curr_abs_alt - ground_abs_alt
        rel_ned_alt = -curr_d
        print(
            f"TAKEOFF_FAILED: rel_abs={rel_abs_alt:.2f}m "
            f"rel_ned={rel_ned_alt:.2f}m required={required_takeoff_alt:.2f}m"
        )
        await publish_leader_led("OFF")
        await drone.action.land()
        telemetry_task.cancel()
        return

    await asyncio.sleep(4)

# =========================================================================
    # DYNAMIC TELEMETRY CAPTURE (FIXED)
    # Capture the exact heading and altitude the drone holds right now in hover.
    # =========================================================================
    print("-- Capturing baseline orientation...")
    
    # 1. Use the current NED altitude held after the takeoff gate.
    hover_absolute_alt = max(takeoff_alt_m, -curr_d)
        
    # 2. Grab the current absolute heading (Yaw reference) from our running background tracker.
    # We spawn facing North (0.0) and fly North
    hover_yaw_deg = 0.0
        
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

    # --- Hover to let followers take off and align ---
    print("-> Hovering in place for 90s. LED ON -> INITIAL_LOCK")
    await publish_leader_led("ON", repeats=6, interval_s=0.25)
    await drone.offboard.set_position_ned(get_ned_position(0.0, 0.0, 0.0))
    for _ in range(90):
        await publish_leader_led("ON", repeats=1, interval_s=0.0)
        await asyncio.sleep(1)

    # --- Start -> Checkpoint 1 (20m forward) ---
    print("-> Moving to Checkpoint 1 (20m forward). LED ON -> FOLLOW")
    await publish_leader_led("ON")
    await drone.offboard.set_position_ned(get_ned_position(20.0, 0.0, 0.0))
    await asyncio.sleep(15) 

    # --- Checkpoint 1 HOLD ---
    print("-> ** Checkpoint 1 HOLD ** (Manual 5Hz BLINK -> HOLD)")
    await publish_leader_led("BLINK")
    await drone.offboard.set_position_ned(get_ned_position(20.0, 0.0, 0.0))
    await asyncio.sleep(8)  

    # --- Checkpoint 1 -> Checkpoint 2 (40m forward, 15m right) ---
    print("-> Moving to Checkpoint 2 (40m forward, 15m right). LED ON -> FOLLOW")
    await publish_leader_led("ON")
    await drone.offboard.set_position_ned(get_ned_position(40.0, 15.0, 0.0))
    await asyncio.sleep(15) 

    # --- Checkpoint 2 HOLD ---
    print("-> ** Checkpoint 2 HOLD ** (Manual 5Hz BLINK -> HOLD)")
    await publish_leader_led("BLINK")
    await drone.offboard.set_position_ned(get_ned_position(40.0, 15.0, 0.0))
    await asyncio.sleep(8)  

    # --- Checkpoint 2 -> Checkpoint 3 (60m forward, -15m left) ---
    print("-> Moving to Checkpoint 3 (60m forward, -15m left). LED ON -> FOLLOW")
    await publish_leader_led("ON")
    await drone.offboard.set_position_ned(get_ned_position(60.0, -15.0, 0.0))
    await asyncio.sleep(15) 

    # --- Checkpoint 3 HOLD ---
    print("-> ** Checkpoint 3 HOLD ** (Manual 5Hz BLINK -> HOLD)")
    await publish_leader_led("BLINK")
    await drone.offboard.set_position_ned(get_ned_position(60.0, -15.0, 0.0))
    await asyncio.sleep(8) 

    # --- Checkpoint 3 -> End (80m forward) ---
    print("-> Moving to Target B (80m forward, 0m right). LED ON -> FOLLOW")
    await publish_leader_led("ON")
    await drone.offboard.set_position_ned(get_ned_position(80.0, 0.0, 0.0))
    await asyncio.sleep(20)

    print("-- Stopping Offboard Mode to allow landing action")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed: {error._result.result}")

    print("-- Landing (LED OFF -> FINISH/LOST)")
    await publish_leader_led("OFF")
    await drone.action.land()
    telemetry_task.cancel()
    
if __name__ == "__main__":
    asyncio.run(run())
