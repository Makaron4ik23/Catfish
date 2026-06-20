import asyncio
import math
from typing import Optional, NamedTuple

import rclpy
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

from .exceptions import ConnectionError, TimeoutError, MAVSDKError
from .bridges import BridgeManager
from .ros_node import DroneROSNode


MAVSDK_UDP_PORT_BASE = 14540
MAVSDK_GRPC_PORT_BASE = 50051
CONNECT_TIMEOUT = 30.0


class PositionNED(NamedTuple):
    north_m: float
    east_m: float
    down_m: float


class Drone:

    def __init__(self, drone_id: int = 0):
        self.drone_id = drone_id
        self._sys: Optional[System] = None
        self._connected = False
        self._ros: Optional[DroneROSNode] = None
        self._bridges: Optional[BridgeManager] = None
        self._ros_initialized = False

    # ── Internal ROS2 init ──────────────────────────────────────────

    def _ensure_ros(self) -> None:
        if self._ros is not None:
            return
        if not self._ros_initialized:
            if not rclpy.ok():
                rclpy.init()
            self._ros_initialized = True
        self._bridges = BridgeManager(self.drone_id)
        self._bridges.start_camera_bridge()
        self._bridges.start_led_bridge()
        self._ros = DroneROSNode(self.drone_id)

    # ── Connection ──────────────────────────────────────────────────

    async def connect(self, timeout: float = CONNECT_TIMEOUT, wait_health: bool = True) -> None:
        udp_port = MAVSDK_UDP_PORT_BASE + self.drone_id
        grpc_port = MAVSDK_GRPC_PORT_BASE + self.drone_id
        address = f'udpin://0.0.0.0:{udp_port}'

        self._sys = System(port=grpc_port)

        try:
            await asyncio.wait_for(
                self._sys.connect(system_address=address),
                timeout=timeout,
            )
        except asyncio.TimeoutError:
            raise TimeoutError(
                f'Drone {self.drone_id} connect to {address} timed out ({timeout}s)'
            )

        try:
            await asyncio.wait_for(
                self._wait_connected(), timeout=timeout,
            )
        except asyncio.TimeoutError:
            raise ConnectionError(
                f'Drone {self.drone_id} did not report connected within {timeout}s'
            )

        if wait_health:
            try:
                await asyncio.wait_for(
                    self._wait_health(), timeout=timeout,
                )
            except asyncio.TimeoutError:
                raise ConnectionError(
                    f'Drone {self.drone_id} health check timed out ({timeout}s)'
                )
        else:
            print(f"[Drone {self.drone_id}] Health gate skipped for SITL startup")

        self._connected = True
        # Disable ALL battery failsafes for SITL simulation, plus GPS-drift
        # arming checks: under heavy SITL load the EKF's stationary
        # horizontal/vertical drift estimate can stay above threshold
        # indefinitely, which would otherwise block arming/health forever.
        # We don't use GPS accuracy for anything (followers fly on vision/LED,
        # the leader only needs local NED for offboard control), so it's safe
        # to relax this check rather than wait on it.
        _bat_params = [
            ("COM_LOW_BAT_ACT", 0),    # no action on low battery
            ("EKF2_GPS_CHECK", 0),     # disable GPS quality/drift arming checks
        ]
        _bat_float_params = [
            ("BAT_LOW_THR", 0.02),      # lower low-battery threshold to 2%
            ("BAT_CRIT_THR", 0.01),     # lower critical threshold to 1%
            ("BAT_EMERGEN_THR", 0.005), # lower emergency threshold to 0.5%
        ]
        for name, val in _bat_params:
            try:
                await self._sys.param.set_param_int(name, val)
            except Exception as e:
                print(f"[Drone {self.drone_id}] Warning: set {name}={val} failed: {e}")
        for name, val in _bat_float_params:
            try:
                await self._sys.param.set_param_float(name, val)
            except Exception as e:
                print(f"[Drone {self.drone_id}] Warning: set {name}={val} failed: {e}")


    async def _wait_connected(self) -> None:
        async for state in self._sys.core.connection_state():
            if state.is_connected:
                return

    async def _wait_health(self) -> None:
        async for health in self._sys.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return

    @property
    def connected(self) -> bool:
        return self._connected

    # ── Basic actions ───────────────────────────────────────────────

    async def arm(self) -> None:
        self._require_connected()
        try:
            await self._sys.action.arm()
        except Exception as e:
            # SITL pre-arm checks (noisy GPS-drift / mag health, already relaxed
            # via params) can deny a normal arm even though the vehicle is fine.
            # Fall back to force-arm, which bypasses pre-arm checks.
            try:
                await self._sys.action.arm_force()
                print(f"[Drone {self.drone_id}] Normal arm denied ({e}); force-armed instead")
            except Exception as e2:
                raise MAVSDKError(f'Failed to arm drone {self.drone_id}: {e2}')

    async def disarm(self) -> None:
        self._require_connected()
        try:
            await self._sys.action.disarm()
        except Exception as e:
            raise MAVSDKError(f'Failed to disarm drone {self.drone_id}: {e}')

    async def takeoff(self, altitude_m: float = 10.0) -> None:
        self._require_connected()
        try:
            await self._sys.action.set_takeoff_altitude(altitude_m)
            await self._sys.action.takeoff()
        except Exception as e:
            raise MAVSDKError(f'Takeoff failed for drone {self.drone_id}: {e}')

    async def land(self) -> None:
        self._require_connected()
        try:
            await self._sys.action.land()
        except Exception as e:
            raise MAVSDKError(f'Land failed for drone {self.drone_id}: {e}')

    async def go_to(self, north: float, east: float, down: float, yaw_deg: float = 0.0, body_frame: bool = False) -> None:
        """Fly to a NED position.

        With body_frame=True, (north, east, down) are body-relative offsets.
        """
        self._require_connected()
        if body_frame:
            pos = await self.position_ned()
            hdg = await self.heading()
            yaw_rad = math.radians(hdg)
            gn = north * math.cos(yaw_rad) - east * math.sin(yaw_rad)
            ge = north * math.sin(yaw_rad) + east * math.cos(yaw_rad)
            target_n = pos.north_m + gn
            target_e = pos.east_m + ge
            target_d = pos.down_m + down
            yaw_deg = hdg
        else:
            target_n = north
            target_e = east
            target_d = down

        try:
            await self._sys.offboard.set_position_ned(
                PositionNedYaw(target_n, target_e, target_d, yaw_deg)
            )
        except OffboardError as e:
            raise MAVSDKError(f'Go_to failed for drone {self.drone_id}: {e}')

    async def move(self, forward: float, right: float, down: float, speed_m_s: float = 5.0, yaw_deg: Optional[float] = None) -> None:
        """Move by a body-relative velocity vector.

        (forward, right, down) define direction in body frame.
        speed_m_s scales the vector magnitude.
        """
        self._require_connected()
        hdg = await self.heading() if yaw_deg is None else yaw_deg
        yaw_rad = math.radians(hdg)
        vn = forward * math.cos(yaw_rad) - right * math.sin(yaw_rad)
        ve = forward * math.sin(yaw_rad) + right * math.cos(yaw_rad)
        vd = down
        norm = math.sqrt(vn*vn + ve*ve + vd*vd)
        if norm > 0.01:
            vn = vn / norm * speed_m_s
            ve = ve / norm * speed_m_s
            vd = vd / norm * speed_m_s
        try:
            await self._sys.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, vd, hdg),
            )
        except OffboardError as e:
            raise MAVSDKError(f'Move failed for drone {self.drone_id}: {e}')

    async def set_velocity(self, north_m_s: float, east_m_s: float, down_m_s: float, yaw_deg: Optional[float] = None) -> None:
        """Set velocity in global NED frame."""
        self._require_connected()
        hdg = await self.heading() if yaw_deg is None else yaw_deg
        try:
            await self._sys.offboard.set_velocity_ned(
                VelocityNedYaw(north_m_s, east_m_s, down_m_s, hdg),
            )
        except OffboardError as e:
            raise MAVSDKError(f'set_velocity failed for drone {self.drone_id}: {e}')

    async def start_offboard(self) -> None:
        self._require_connected()
        pos = await self.position_ned()
        hdg = await self.heading()
        setpoint = PositionNedYaw(pos.north_m, pos.east_m, pos.down_m, 0.0)
        for _ in range(5):
            await self._sys.offboard.set_position_ned(setpoint)
            await asyncio.sleep(0.05)
        try:
            await self._sys.offboard.start()
        except OffboardError as e:
            raise MAVSDKError(f'Offboard start failed for drone {self.drone_id}: {e}')

    async def stop_offboard(self) -> None:
        self._require_connected()
        try:
            await self._sys.offboard.stop()
        except Exception as e:
            raise MAVSDKError(f'Offboard stop failed for drone {self.drone_id}: {e}')

    async def set_takeoff_altitude(self, altitude_m: float) -> None:
        self._require_connected()
        await self._sys.action.set_takeoff_altitude(altitude_m)

    # ── Telemetry ───────────────────────────────────────────────────

    async def position_ned(self) -> PositionNED:
        self._require_connected()
        async for pos in self._sys.telemetry.position_velocity_ned():
            return PositionNED(
                pos.position.north_m, pos.position.east_m, pos.position.down_m
            )

    async def heading(self) -> float:
        # telemetry.heading() is derived from the global position estimate, which
        # never becomes valid in this GPS-less world, so it would block forever.
        # attitude_euler.yaw_deg is the same heading (0 = North in NED) and comes
        # straight from the attitude estimator, which is always available.
        self._require_connected()
        async for att in self._sys.telemetry.attitude_euler():
            return att.yaw_deg % 360.0

    # ── Telemetry Communication ─────────────────────────────────────

    def publish_telemetry(
        self,
        n: float,
        e: float,
        d: float,
        vn: float,
        ve: float,
        vd: float,
        yaw: float,
        abs_alt: float,
        airborne: Optional[bool] = None,
        target_id: Optional[int] = None,
    ) -> None:
        self._ensure_ros()
        import json
        import time
        is_airborne = bool(-float(d) > 1.5) if airborne is None else bool(airborne)
        data = {
            "id": int(self.drone_id),
            "t": time.time(),
            "n": float(n),
            "e": float(e),
            "d": float(d),
            "vn": float(vn),
            "ve": float(ve),
            "vd": float(vd),
            "yaw": float(yaw),
            "abs_alt": float(abs_alt),
            "airborne": is_airborne,
        }
        if target_id is not None:
            data["target_id"] = int(target_id)
        self._ros.publish_telemetry(json.dumps(data))

    def get_target_telemetry(self, target_id: Optional[int] = None, max_age_s: float = 1.0) -> Optional[dict]:
        self._ensure_ros()
        return self._ros.target_telemetry(target_id=target_id, max_age_s=max_age_s)

    # ── LED control ─────────────────────────────────────────────────

    def set_leds(self, mask: str) -> None:
        self._ensure_ros()
        self._ros.publish_led(mask)

    def led_on(self) -> None:
        self._ensure_ros()
        self._ros.publish_led('ON')

    def led_off(self) -> None:
        self._ensure_ros()
        self._ros.publish_led('OFF')

    def led_blink(self) -> None:
        self._ensure_ros()
        self._ros.publish_led('BLINK')

    # ── Camera ──────────────────────────────────────────────────────

    def start_camera(self) -> None:
        """Start bridges + ROS node (no background spin)."""
        self._ensure_ros()

    def stop_camera(self) -> None:
        if self._ros:
            self._ros.stop_spin()
        if self._bridges:
            self._bridges.stop_all()

    def camera_frame(self):
        """Get the latest frame (call spin() first to process callbacks)."""
        if self._ros is None:
            return None
        return self._ros.frame()

    def spin(self) -> None:
        """Process one ROS2 callback inline (call before camera_frame())."""
        if self._ros is None:
            return
        self._ros.spin_once()

    # ── Cleanup ─────────────────────────────────────────────────────

    async def close(self) -> None:
        self.stop_camera()
        self._connected = False
        self._sys = None

    def _require_connected(self) -> None:
        if not self._connected or self._sys is None:
            raise ConnectionError(f'Drone {self.drone_id} not connected')
