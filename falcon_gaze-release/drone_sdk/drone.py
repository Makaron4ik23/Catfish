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

    async def connect(self, timeout: float = CONNECT_TIMEOUT) -> None:
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

        try:
            await asyncio.wait_for(
                self._wait_health(), timeout=timeout,
            )
        except asyncio.TimeoutError:
            raise ConnectionError(
                f'Drone {self.drone_id} health check timed out ({timeout}s)'
            )

        self._connected = True
        try:
            await self._sys.param.set_param_int("COM_LOW_BAT_ACT", 0)
        except Exception as e:
            print(f"[Drone {self.drone_id}] Warning: failed to disable battery failsafe: {e}")


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
            raise MAVSDKError(f'Failed to arm drone {self.drone_id}: {e}')

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
        self._require_connected()
        async for hdg in self._sys.telemetry.heading():
            return hdg.heading_deg

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
