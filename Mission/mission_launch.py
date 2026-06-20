import argparse
import asyncio
import json
import math
import os
import sys
from pathlib import Path

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "falcon_gaze-release"))
import rclpy
from drone_sdk import Drone


DEFAULT_WAYPOINT_FILE = Path("px4_mavsdk_waypoints.json")
DEFAULT_CONNECTION = "udpin://0.0.0.0:14540"
TAKEOFF_ALT_M = 5.0   # match the followers' takeoff altitude so their forward
                      # cameras keep the leader's LED in frame for the visual lock
LOOP_RATE_HZ = 10.0
DESIRED_SPEED_MPS = 4.0
FINAL_HOLD_SECONDS = 5.0


def wrap_degrees(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def load_waypoints(path):
    with path.open("r", encoding="utf-8") as file:
        mission = json.load(file)

    waypoints = []
    for item in mission["waypoints"]:
        ned = item["px4_ned"]
        waypoints.append(
            {
                "north_m": float(ned["north_m"]),
                "east_m": float(ned["east_m"]),
                "down_m": float(ned["down_m"]),
                "yaw_deg": float(item.get("yaw_deg", 0.0)),
                "speed_to_next_mps": item.get("speed_to_next_mps"),
                "source_index": item.get("source_index"),
            }
        )

    if not waypoints:
        raise ValueError("Mission file has no waypoints.")

    return waypoints


def distance_3d(start_wp, end_wp):
    dn = end_wp["north_m"] - start_wp["north_m"]
    de = end_wp["east_m"] - start_wp["east_m"]
    dd = end_wp["down_m"] - start_wp["down_m"]
    return math.sqrt(dn**2 + de**2 + dd**2)


def speed_for_leg(start_wp, default_speed_mps):
    speed = start_wp.get("speed_to_next_mps")
    if speed is None:
        return default_speed_mps

    speed = float(speed)
    if speed <= 0.0:
        raise ValueError("speed_to_next_mps must be greater than 0.")

    return speed


def interpolate_waypoint(start_wp, end_wp, t):
    return {
        "north_m": start_wp["north_m"] + ((end_wp["north_m"] - start_wp["north_m"]) * t),
        "east_m": start_wp["east_m"] + ((end_wp["east_m"] - start_wp["east_m"]) * t),
        "down_m": start_wp["down_m"] + ((end_wp["down_m"] - start_wp["down_m"]) * t),
        "yaw_deg": start_wp["yaw_deg"] + ((end_wp["yaw_deg"] - start_wp["yaw_deg"]) * t),
    }


def path_yaw_deg(start_wp, end_wp):
    north_delta = end_wp["north_m"] - start_wp["north_m"]
    east_delta = end_wp["east_m"] - start_wp["east_m"]
    if abs(north_delta) < 1e-6 and abs(east_delta) < 1e-6:
        return start_wp["yaw_deg"]
    return wrap_degrees(math.degrees(math.atan2(east_delta, north_delta)))


async def first_telemetry_value(stream):
    async for value in stream:
        return value
    raise RuntimeError("Telemetry stream ended before returning a value.")


async def wait_for_connection(drone):
    print("Waiting for PX4 connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("PX4 discovered.")
            return


async def relax_sitl_arming_checks(drone):
    """Disable noisy SITL failsafes/GPS-drift checks that can block arming/health
    indefinitely under load. We don't rely on GPS accuracy for local NED offboard
    control, so EKF2_GPS_CHECK and battery/mag failsafes are safe to relax here.
    """
    for name, val in [
        ("COM_LOW_BAT_ACT", 0),
        ("SYS_HAS_MAG", 0),
        ("COM_ARM_MAG_STR", 0),
        ("EKF2_GPS_CHECK", 0),
        ("COM_ARM_WO_GPS", 1),
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


async def wait_until_ready(drone, timeout_s=20.0):
    """Best-effort wait for a usable position estimate. Bounded: under SITL load
    the GPS-drift health flags can stay false indefinitely even though home
    position and local NED are already usable, so we don't block forever on them
    (the old, proven leader script never waited on telemetry.health() at all).
    """
    print(f"Waiting up to {timeout_s:.0f}s for global/home position estimate...")

    async def _wait_health_ok():
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return

    try:
        await asyncio.wait_for(_wait_health_ok(), timeout=timeout_s)
        print("Estimator ready.")
    except asyncio.TimeoutError:
        print("Estimator not confirmed ready within timeout — proceeding anyway (home position is set; local NED control does not need GPS-quality flags).")


async def arm_with_fallback(drone, attempts=6, retry_delay_s=3.0):
    """Arm the leader robustly under SITL.

    Tries a normal arm() first; on COMMAND_DENIED (noisy GPS-drift / mag health
    that we've already relaxed via params) it falls back to arm_force(), which
    bypasses pre-arm checks. Retries a few times so a transient "Resolve system
    health failures first" while the EKF settles does not abort the mission.
    """
    from mavsdk.action import ActionError

    last_exc = None
    for attempt in range(1, attempts + 1):
        try:
            await drone.action.arm()
            print(f"-- Armed (normal) on attempt {attempt}")
            return
        except ActionError as exc:
            last_exc = exc
            print(f"   arm() denied (attempt {attempt}/{attempts}): {exc._result.result}. Trying force-arm...")
            try:
                await drone.action.arm_force()
                print(f"-- Armed (forced) on attempt {attempt}")
                return
            except ActionError as exc2:
                last_exc = exc2
                print(f"   arm_force() failed (attempt {attempt}/{attempts}): {exc2._result.result}")
        await asyncio.sleep(retry_delay_s)
    raise RuntimeError(f"Could not arm leader after {attempts} attempts: {last_exc}")


async def send_position(drone, waypoint, hover_absolute_alt_m, yaw_deg):
    # Converted waypoints store down_m as an offset from the first Blender point.
    # MAVSDK PositionNedYaw needs down relative to PX4 local origin, so offset it
    # by the real altitude captured after takeoff.
    absolute_down_m = -hover_absolute_alt_m + waypoint["down_m"]
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            waypoint["north_m"],
            waypoint["east_m"],
            absolute_down_m,
            wrap_degrees(yaw_deg),
        )
    )


def _publish_led(led_drone, mode):
    """Best-effort LED state push (followers decode FOLLOW/HOLD/LOST from this)."""
    if mode == "ON":
        led_drone.led_on()
    elif mode == "OFF":
        led_drone.led_off()
    elif mode == "BLINK":
        led_drone.led_blink()
    led_drone.spin()


async def fly_mission(args):
    mission_path = args.waypoints.resolve()
    waypoints = load_waypoints(mission_path)

    rclpy.init()
    led_drone = Drone(drone_id=0)

    drone = System()
    await drone.connect(system_address=args.connection)

    await wait_for_connection(drone)
    await relax_sitl_arming_checks(drone)
    await wait_until_ready(drone)

    health = await first_telemetry_value(drone.telemetry.health())
    print(f"-- Pre-arm health: {health}")

    print("-- Arming x500")
    await arm_with_fallback(drone)
    await asyncio.sleep(2)

    # ── Offboard takeoff ───────────────────────────────────────────
    # AUTO takeoff (action.takeoff) needs a global-position fix this world's x500
    # never gets (is_global_position_ok stays False), so it won't climb. Instead
    # climb with OFFBOARD position setpoints in LOCAL NED (is_local_position_ok is
    # True), which is the same control path the mission legs already use.
    print(f"-- Offboard takeoff to {args.takeoff_alt}m")

    async def _first_ned():
        async for pv in drone.telemetry.position_velocity_ned():
            return pv

    pv0 = await asyncio.wait_for(_first_ned(), timeout=15)
    start_n, start_e, start_d = (
        pv0.position.north_m,
        pv0.position.east_m,
        pv0.position.down_m,
    )
    target_d = start_d - args.takeoff_alt   # NED down is negative up → subtract to climb

    # Heading from the attitude estimator (attitude_euler.yaw_deg, 0 = North).
    # telemetry.heading() depends on the global-position estimate this GPS-less
    # world never produces, so it would block; fall back to spawn heading (0).
    async def _first_heading():
        async for att in drone.telemetry.attitude_euler():
            return att.yaw_deg % 360.0

    try:
        captured_yaw_deg = await asyncio.wait_for(_first_heading(), timeout=8)
    except asyncio.TimeoutError:
        captured_yaw_deg = 0.0
        print("Heading stream slow — defaulting captured yaw to 0 deg (spawn heading).")

    # Prime the offboard stream with the climb setpoint, then engage offboard.
    climb_sp = PositionNedYaw(start_n, start_e, target_d, wrap_degrees(captured_yaw_deg))
    for _ in range(10):
        await drone.offboard.set_position_ned(climb_sp)
        await asyncio.sleep(0.05)

    print("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Hold the climb setpoint until ~90% of target altitude (or timeout).
    loop = asyncio.get_event_loop()
    climb_deadline = loop.time() + 40.0
    while loop.time() < climb_deadline:
        await drone.offboard.set_position_ned(climb_sp)
        pv = await _first_ned()
        if -pv.position.down_m >= args.takeoff_alt * 0.9:
            break
        await asyncio.sleep(0.2)

    pv = await _first_ned()
    hover_absolute_alt_m = -pv.position.down_m
    print(
        f"Hover reference captured: altitude={hover_absolute_alt_m:.2f}m, "
        f"yaw={captured_yaw_deg:.2f} deg"
    )

    first_wp = waypoints[0]
    initial_yaw = select_yaw(args.yaw_mode, first_wp, first_wp, captured_yaw_deg)
    await send_position(drone, first_wp, hover_absolute_alt_m, initial_yaw)
    print("-- Offboard active, beginning mission")

    print("-- LED ON (FOLLOW signal for followers)")
    for _ in range(6):
        _publish_led(led_drone, "ON")
        await asyncio.sleep(0.1)

    dt = 1.0 / args.loop_rate

    try:
        print(f"-> Flying {len(waypoints)} converted Blender/PX4 waypoints")
        for index in range(len(waypoints) - 1):
            start_wp = waypoints[index]
            end_wp = waypoints[index + 1]
            segment_distance = distance_3d(start_wp, end_wp)
            segment_speed = speed_for_leg(start_wp, args.speed)
            segment_duration = segment_distance / segment_speed
            steps = max(1, math.ceil(segment_duration * args.loop_rate))
            yaw_deg = select_yaw(args.yaw_mode, start_wp, end_wp, captured_yaw_deg)

            print(
                f"Leg {index} -> {index + 1} | "
                f"distance={segment_distance:.2f}m | "
                f"speed={segment_speed:.2f}m/s | "
                f"yaw={yaw_deg:.2f} deg"
            )

            for step in range(1, steps + 1):
                waypoint = interpolate_waypoint(start_wp, end_wp, step / steps)
                await send_position(drone, waypoint, hover_absolute_alt_m, yaw_deg)
                _publish_led(led_drone, "ON")
                await asyncio.sleep(dt)

        final_wp = waypoints[-1]
        final_yaw = select_yaw(args.yaw_mode, final_wp, final_wp, captured_yaw_deg)
        hold_steps = max(1, math.ceil(args.final_hold * args.loop_rate))

        print("-> Final waypoint reached. Holding position.")
        for _ in range(hold_steps):
            await send_position(drone, final_wp, hover_absolute_alt_m, final_yaw)
            _publish_led(led_drone, "ON")
            await asyncio.sleep(dt)

    finally:
        print("-- LED OFF (FINISH/LOST signal for followers)")
        for _ in range(6):
            _publish_led(led_drone, "OFF")
            await asyncio.sleep(0.1)

        print("-- Stopping offboard mode")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed: {error._result.result}")

        print("-- Landing")
        await drone.action.land()


def select_yaw(yaw_mode, start_wp, end_wp, captured_yaw_deg):
    if yaw_mode == "current":
        return captured_yaw_deg
    if yaw_mode == "path":
        return path_yaw_deg(start_wp, end_wp)
    return start_wp["yaw_deg"]


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Fly converted Blender mesh/spline waypoints in PX4 Gazebo x500."
    )
    parser.add_argument(
        "waypoints",
        nargs="?",
        type=Path,
        default=DEFAULT_WAYPOINT_FILE,
        help="Converted JSON from convert_blender_vertices_to_px4.py",
    )
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument("--takeoff-alt", type=float, default=TAKEOFF_ALT_M)
    parser.add_argument("--takeoff-wait", type=float, default=10.0)
    parser.add_argument("--speed", type=float, default=DESIRED_SPEED_MPS)
    parser.add_argument("--loop-rate", type=float, default=LOOP_RATE_HZ)
    parser.add_argument("--final-hold", type=float, default=FINAL_HOLD_SECONDS)
    parser.add_argument(
        "--yaw-mode",
        choices=["file", "current", "path"],
        default="current",
        help=(
            "file = yaw from converted JSON, current = keep takeoff heading, "
            "path = face each path segment"
        ),
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    asyncio.run(fly_mission(args))


if __name__ == "__main__":
    main()
