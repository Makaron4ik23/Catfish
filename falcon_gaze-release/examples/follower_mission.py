#!/usr/bin/env python3
"""
Follower mission — vision-based leader tracking for the GENERA hackathon.

Each follower drone uses its front-facing camera to detect the green LED
beacon on the drone ahead and follows it with a PD-controller.

State machine:
    TAKEOFF → FOLLOW/HOLD ⇄ YAW_SEARCH → LAND

Additionally integrates:
- Stateful hysteresis anti-collision with direct CRITICAL_AVOID jumps.
- Lateral stagger offset to prevent occlusion on sharp turns.
- Masked central obstacle avoidance with dynamic direction selection.
- Telemetry pitch-tilt visual compensation.
- Solidity shape filtering on contours to reject sun/tree reflections.
"""

import asyncio
import collections
import math
import os
import signal
import sys
import threading
import time
from enum import Enum

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2
import numpy as np
import rclpy
from drone_sdk import Drone

# ══════════════════════════════════════════════════════════════════════
#  Configuration
# ══════════════════════════════════════════════════════════════════════

FOLLOWER_IDS = [1, 2, 3]

# Flight
TARGET_ABS_ALT = 42.0   # Target absolute altitude (MSL) for the entire swarm
MIN_COMMAND_TAKEOFF_ALT = 5.0
TAKEOFF_WAIT_S = 8      # stabilisation time after takeoff (longer for slow SITL)
TAKEOFF_TIMEOUT_S = 35.0
TAKEOFF_RETRY_AFTER_S = 10.0
TAKEOFF_RETRY_LIMIT = 1
MIN_TAKEOFF_REL_ALT = 2.0
FOLLOWER_START_STAGGER_S = 20.0

# Camera
FRAME_W, FRAME_H = 640, 480
CX, CY = FRAME_W / 2, FRAME_H / 2      # frame centre
_SHOW_CAMERA_RAW = os.environ.get("SHOW_CAMERA", "auto").strip().lower()
SHOW_CAMERA = (
    _SHOW_CAMERA_RAW not in {"0", "false", "no", "off"}
    and (
        _SHOW_CAMERA_RAW in {"1", "true", "yes", "on"}
        or bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    )
)
try:
    CAMERA_DISPLAY_SCALE = float(os.environ.get("CAMERA_DISPLAY_SCALE", "0.75"))
except ValueError:
    CAMERA_DISPLAY_SCALE = 0.75

# HSV thresholds for the green LED (calibrated and expanded for robust tracking)
HSV_LO = np.array([40, 140, 160])
HSV_HI = np.array([72, 255, 255])

# Minimum/maximum contour area (pixels²)
MIN_AREA = 8.0
MAX_ALLOWED_AREA = 950.0
TRACK_MIN_AREA = 20.0
TRACK_MIN_W = 6
TRACK_MIN_H = 5
INITIAL_LOCK_MIN_AREA = 45.0
INITIAL_LOCK_MIN_W = 8
INITIAL_LOCK_MIN_H = 6
INITIAL_LOCK_FRAMES = 5

# Blob shape filters
MIN_BLOB_DIM  = 3     # pixels
MAX_BLOB_DIM  = 48    # pixels (reject big structures like trees/houses, leaves headroom for w~28)
MAX_ASPECT    = 4.0   # w/h ratio — reject long horizontal strips (grass edges)



# Solidity filter limits (for contours >= SOLIDITY_MIN_AREA)
SOLIDITY_MIN_AREA = 50.0
SOLIDITY_THRESHOLD = 0.82

# Target LED parameters for distance estimation
TARGET_AREA = 600.0   # Area-based target (used when stagger is active)
TARGET_W = 18.0

# ── Controller Gains ───────────────────────────────────────────────
KP_YAW     = 40.0      # yaw correction gain (tighter tracking)
KD_YAW     = 4.0       # yaw derivative gain (more damping)
KP_FORWARD = 6.0       # forward speed gain (smoother chain follow)
KD_FORWARD = 0.8       # forward speed derivative gain (dampen chain oscillation)
KP_ALT     = 1.5       # climb rate gain
ALPHA_D    = 0.4       # exponential smoothing factor for derivative

# ── Collision Avoidance Thresholds ─────────────────────────────────
COLLISION_W_ENTER = 23.0
COLLISION_W_EXIT  = 20.0
CRITICAL_W_ENTER  = 28.0
CRITICAL_W_EXIT   = 25.0

# ── Speed clamps ───────────────────────────────────────────────────
MAX_FWD  = 8.0   # m/s (increased to allow keeping up with leader)
MAX_VERT = 1.0   # m/s

# ── Timing ─────────────────────────────────────────────────────────
INITIAL_SEARCH_TIMEOUT = 120.0  # seconds to search before first LED lock
SEARCH_RATE    = 8.0            # deg/s yaw rotation in search mode
TARGET_SEARCH_RATE = 30.0       # deg/s when telemetry gives target bearing
TARGET_SEARCH_MAX_AGE_S = 2.0
TARGET_SEARCH_MIN_RANGE_M = 0.5
LAND_TIMEOUT   = 30.0           # seconds before auto-land (more patience in chain)
CTRL_DT        = 0.05           # 20 Hz control loop
LOG_INTERVAL   = 2.0            # seconds between periodic debug prints
TELEMETRY_MAX_AGE_S = 1.0


# ══════════════════════════════════════════════════════════════════════
#  State machine
# ══════════════════════════════════════════════════════════════════════

class State(Enum):
    TAKEOFF    = "TAKEOFF"
    FOLLOW     = "FOLLOW"
    HOLD       = "HOLD"
    SAFE_HOVER = "SAFE_HOVER"
    YAW_SEARCH = "YAW_SEARCH"
    LAND       = "LAND"


# ══════════════════════════════════════════════════════════════════════
#  Pure helper functions
# ══════════════════════════════════════════════════════════════════════

def update_collision_state(curr_state, w):
    """Update and return collision safety state using hysteresis."""
    if w > CRITICAL_W_ENTER:
        return "CRITICAL_AVOID"
    elif curr_state == "CRITICAL_AVOID" and w < CRITICAL_W_EXIT:
        return "COLLISION_AVOID"
    elif curr_state != "CRITICAL_AVOID" and w > COLLISION_W_ENTER:
        return "COLLISION_AVOID"
    elif curr_state == "COLLISION_AVOID" and w < COLLISION_W_EXIT:
        return "NORMAL"
    return curr_state


def _rate_limit(target, prev, max_accel, dt):
    """Limit acceleration to smooth out command jumps."""
    max_step = max_accel * dt
    return float(np.clip(target, prev - max_step, prev + max_step))


def update_pd_filter(err, prev_err, prev_err_diff, alpha, dt):
    """Compute and smooth derivative error component."""
    raw_diff = (err - prev_err) / dt
    smoothed_diff = alpha * raw_diff + (1 - alpha) * prev_err_diff
    return smoothed_diff


def angle_diff_deg(target_deg, current_deg):
    """Return signed shortest heading error in degrees."""
    return (target_deg - current_deg + 540.0) % 360.0 - 180.0


def step_heading_towards(current_deg, target_deg, max_step_deg):
    """Move current heading toward target heading by at most max_step_deg."""
    err = angle_diff_deg(target_deg, current_deg)
    step = float(np.clip(err, -max_step_deg, max_step_deg))
    return (current_deg + step) % 360.0


def compute_obstacle_density(frame, led_bbox=None, box_size=160, pitch_comp=0.0):
    """Compute edge density in the center ROI, masking the target LED and ground."""
    cx0, cy0 = FRAME_W // 2 - box_size // 2, FRAME_H // 2 - box_size // 2
    roi = frame[cy0:cy0+box_size, cx0:cx0+box_size].copy()

    # 1. Mask out region around LED target if visible (scaled to block out the drone body)
    if led_bbox is not None:
        lx, ly, lw, lh = led_bbox
        pad_x = int(2.5 * lw)
        pad_y = int(1.2 * lh)
        mx0 = max(0, lx - pad_x - cx0)
        my0 = max(0, ly - pad_y - cy0)
        mx1 = min(box_size, lx + lw + pad_x - cx0)
        my1 = min(box_size, ly + lh + pad_y - cy0)
        if mx1 > mx0 and my1 > my0:
            roi[my0:my1, mx0:mx1] = 0

    # 2. Mask out the ground (anything below the pitch-compensated horizon line)
    # Horizon is at Y = CY + pitch_comp. We add a safety margin of +15px.
    horizon_rel_y = int(CY + pitch_comp + 15 - cy0)
    if 0 <= horizon_rel_y < box_size:
        roi[horizon_rel_y:, :] = 0
    elif horizon_rel_y < 0:
        roi[:, :] = 0

    edges = cv2.Canny(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), 50, 150)
    return float(np.count_nonzero(edges) / edges.size)


def obstacle_avoid_direction(frame, box_size=160):
    """Determine slide direction based on left/right split ROI edge density."""
    cx0, cy0 = FRAME_W // 2 - box_size // 2, FRAME_H // 2 - box_size // 2
    roi = frame[cy0:cy0+box_size, cx0:cx0+box_size]
    edges = cv2.Canny(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), 50, 150)
    left_density = np.count_nonzero(edges[:, :box_size // 2])
    right_density = np.count_nonzero(edges[:, box_size // 2:])
    # Avoid towards the side with lower density
    return -1 if left_density > right_density else 1


def detect_led(frame, state=None):
    """Return (cx, cy, w, h, area) of the largest green blob, or None."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Adaptive thresholds during active search to handle motion blur smearing
    if state == State.YAW_SEARCH:
        lo = np.array([42, 110, 160])  # loosen saturation/value
        hi = np.array([68, 255, 255])
        max_aspect = 8.0               # allow elongated blobs due to motion blur
    else:
        lo = HSV_LO
        hi = HSV_HI
        max_aspect = MAX_ASPECT
        
    mask = cv2.inRange(hsv, lo, hi)

    # morphological cleanup
    _morph_kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, _morph_kernel, iterations=1)
    mask = cv2.dilate(mask, _morph_kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0.0
    for c in contours:
        a = cv2.contourArea(c)
        if a < MIN_AREA or a > MAX_ALLOWED_AREA:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        # reject tiny or large blobs (trees, structures)
        if bw < MIN_BLOB_DIM or bh < MIN_BLOB_DIM:
            continue
        if bw > MAX_BLOB_DIM or bh > MAX_BLOB_DIM:
            continue
        # reject elongated horizontal strips (grass edges)
        aspect = bw / max(bh, 1)
        if aspect > max_aspect:
            continue
            
        # Solidity shape filtering (ignore on tiny contours to prevent noise edge errors)
        if a >= SOLIDITY_MIN_AREA:
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = a / hull_area if hull_area > 0 else 0.0
            if solidity < SOLIDITY_THRESHOLD:
                continue
                
        if a > best_area:
            best, best_area = c, a


    if best is None:
        return None

    x, y, w, h = cv2.boundingRect(best)
    M = cv2.moments(best)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = x + w // 2, y + h // 2
    return cx, cy, w, h, best_area


def is_tracking_candidate(det):
    """Return True when a detected blob is large enough for active tracking."""
    if det is None:
        return False
    _cx, _cy, w, h, area = det
    return area >= TRACK_MIN_AREA and w >= TRACK_MIN_W and h >= TRACK_MIN_H


def is_initial_lock_candidate(det):
    """Use a stricter gate before the first lock to reject tiny reflections."""
    if det is None:
        return False
    _cx, _cy, w, h, area = det
    return area >= INITIAL_LOCK_MIN_AREA and w >= INITIAL_LOCK_MIN_W and h >= INITIAL_LOCK_MIN_H


def is_usable_target_telemetry(target_data):
    if target_data is None:
        return False
    return bool(target_data.get("airborne", False))


def draw_camera_overlay(frame, drone_id, state, det, initial_lock, protocol_cmd,
                        target_id, yaw_deg, pitch_deg):
    """Return a debug camera frame with tracking state and LED annotation."""
    view = frame.copy()
    fh, fw = view.shape[:2]
    led_label = "NO_LED"
    led_color = (0, 0, 255)

    if det is not None:
        cx, cy, w, h, area = det
        x0 = max(0, int(cx - w / 2))
        y0 = max(0, int(cy - h / 2))
        x1 = min(fw - 1, int(cx + w / 2))
        y1 = min(fh - 1, int(cy + h / 2))
        led_color = (0, 255, 0)
        led_label = f"LED area={area:.0f}"
        cv2.rectangle(view, (x0, y0), (x1, y1), led_color, 2)
        cv2.drawMarker(view, (int(cx), int(cy)), led_color,
                       markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)

    cv2.line(view, (int(CX), 0), (int(CX), fh - 1), (255, 255, 255), 1)
    cv2.line(view, (0, int(CY)), (fw - 1, int(CY)), (255, 255, 255), 1)
    cv2.rectangle(view, (0, 0), (fw, 58), (0, 0, 0), -1)
    cv2.putText(
        view,
        f"D{drone_id} {state.value} {protocol_cmd} lock={int(initial_lock)} tgt={target_id}",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        view,
        f"{led_label} yaw={yaw_deg:.1f} pitch={pitch_deg:.1f}",
        (8, 46),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        led_color,
        1,
        cv2.LINE_AA,
    )
    return view


def show_camera_frame(drone_id, frame, state, det, initial_lock, protocol_cmd,
                      target_id, yaw_deg, pitch_deg):
    view = draw_camera_overlay(
        frame, drone_id, state, det, initial_lock, protocol_cmd,
        target_id, yaw_deg, pitch_deg,
    )
    if CAMERA_DISPLAY_SCALE > 0 and abs(CAMERA_DISPLAY_SCALE - 1.0) > 1e-3:
        view = cv2.resize(
            view,
            None,
            fx=CAMERA_DISPLAY_SCALE,
            fy=CAMERA_DISPLAY_SCALE,
            interpolation=cv2.INTER_AREA,
        )
    cv2.imshow(f"Drone {drone_id} camera", view)
    return cv2.waitKey(1) & 0xFF


# ══════════════════════════════════════════════════════════════════════
#  Camera-spin background thread (shared by all drones)
# ══════════════════════════════════════════════════════════════════════

def _camera_spin(drones, stop_evt):
    for d in drones:
        d.start_camera()
    while not stop_evt.is_set() and rclpy.ok():
        for d in drones:
            d.spin()
        time.sleep(0.001)
    for d in drones:
        d.stop_camera()


# ══════════════════════════════════════════════════════════════════════
#  Per-drone follower coroutine
# ══════════════════════════════════════════════════════════════════════

async def follower(drone, drone_id, shutdown, start_delay=0.0, camera_status=None):
    _log = lambda msg: print(f"[Drone {drone_id}] {msg}")
    if start_delay > 0.0:
        _log(f"Staggered startup: waiting {start_delay:.1f}s before arming...")
        await asyncio.sleep(start_delay)

    state = State.TAKEOFF
    protocol_cmd = "LOST"
    initial_lock = False
    initial_lock_hits = 0
    airborne = False
    hold_abs_alt = None
    last_det_t = time.time()
    last_dir   = 1          # +1 = LED was to the right, −1 = left
    last_log_t = 0.0        # timestamp of last periodic debug print
    last_img_t = 0.0        # timestamp of last image capture

    # For protocol decoding
    det_history = collections.deque(maxlen=int(2.0 / CTRL_DT))  # 2 seconds history

    # For tracking gate
    last_cx, last_cy = None, None
    last_det_time = 0.0

    # Stateful safety parameters
    collision_state = "NORMAL"
    prev_fwd_speed = 0.0

    # PD controller memory
    prev_err_d = 0.0
    prev_err_d_diff = 0.0
    prev_err_x = 0.0
    prev_err_x_diff = 0.0

    last_vel_log_t = 0.0

    def active_target_id():
        if drone_id <= 1 or not initial_lock:
            return 0
        previous_data = drone.get_target_telemetry(
            target_id=drone_id - 1,
            max_age_s=TARGET_SEARCH_MAX_AGE_S,
        )
        if is_usable_target_telemetry(previous_data):
            return drone_id - 1
        return 0

    def active_target_bearing():
        target_data = drone.get_target_telemetry(
            target_id=active_target_id(),
            max_age_s=TARGET_SEARCH_MAX_AGE_S,
        )
        if not is_usable_target_telemetry(target_data):
            return None

        dn = target_data["n"] - curr_n
        de = target_data["e"] - curr_e
        if math.hypot(dn, de) < TARGET_SEARCH_MIN_RANGE_M:
            return None
        return math.degrees(math.atan2(de, dn)) % 360.0

    # ── Pitch Telemetry background subscription ────────────────────
    current_pitch_deg = 0.0
    async def track_pitch():
        nonlocal current_pitch_deg
        while not shutdown.is_set():
            try:
                async for att in drone._sys.telemetry.attitude_euler():
                    current_pitch_deg = att.pitch_deg
            except asyncio.CancelledError:
                break
            except Exception as exc:
                _log(f"Telemetry pitch stream lost: {exc}, reconnecting...")
                await asyncio.sleep(0.5)

    pitch_task = asyncio.create_task(track_pitch())

    # ── High-Speed Telemetry background tracking ─────────────────
    curr_n, curr_e, curr_d = 0.0, 0.0, 0.0
    curr_vn, curr_ve, curr_vd = 0.0, 0.0, 0.0
    curr_yaw = 0.0
    curr_abs_alt = None   # height above local origin (-down_m); None until first NED sample

    def update_camera_status(det=None):
        if camera_status is None:
            return
        camera_status[drone_id] = {
            "state": state,
            "det": det,
            "initial_lock": initial_lock,
            "protocol_cmd": protocol_cmd,
            "target_id": active_target_id(),
            "yaw": curr_yaw,
            "pitch": current_pitch_deg,
        }

    async def track_telemetry_loop():
        nonlocal curr_n, curr_e, curr_d, curr_vn, curr_ve, curr_vd, curr_yaw, curr_abs_alt
        async def fetch_heading():
            # telemetry.heading() needs the global position estimate (never valid
            # in this GPS-less world) and would block forever. attitude_euler.yaw_deg
            # is the same heading (0 = North in NED) from the attitude estimator.
            nonlocal curr_yaw
            while not shutdown.is_set():
                try:
                    async for att in drone._sys.telemetry.attitude_euler():
                        curr_yaw = att.yaw_deg % 360.0
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    _log(f"Telemetry heading stream lost: {e}, reconnecting...")
                    await asyncio.sleep(0.5)
        async def fetch_abs_alt():
            # This world's x500 has no GPS fix (is_global_position_ok stays False),
            # so telemetry.position() (GPS/MSL) never yields. Use LOCAL NED instead:
            # curr_abs_alt becomes height above the PX4 local origin (-down_m). All
            # consumers use it relatively (rel_abs_alt, hold_abs_alt, target abs_alt),
            # and every drone's origin sits at the same platform level, so this stays
            # consistent across the swarm.
            nonlocal curr_abs_alt
            while not shutdown.is_set():
                try:
                    async for pv in drone._sys.telemetry.position_velocity_ned():
                        curr_abs_alt = -pv.position.down_m
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    _log(f"Telemetry alt stream lost: {e}, reconnecting...")
                    await asyncio.sleep(0.5)
        async def fetch_pos_vel():
            nonlocal curr_n, curr_e, curr_d, curr_vn, curr_ve, curr_vd
            while not shutdown.is_set():
                try:
                    async for pv in drone._sys.telemetry.position_velocity_ned():
                        curr_n = pv.position.north_m
                        curr_e = pv.position.east_m
                        curr_d = pv.position.down_m
                        curr_vn = pv.velocity.north_m_s
                        curr_ve = pv.velocity.east_m_s
                        curr_vd = pv.velocity.down_m_s
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    _log(f"Telemetry pos stream lost: {e}, reconnecting...")
                    await asyncio.sleep(0.5)

        tasks = [
            asyncio.create_task(fetch_heading()),
            asyncio.create_task(fetch_abs_alt()),
            asyncio.create_task(fetch_pos_vel())
        ]
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            pass
        finally:
            for t in tasks:
                t.cancel()

    # ── Velocity control interception wrapper ───────────────────
    async def send_velocity(vn_cmd, ve_cmd, vert_spd_cmd, yaw_deg_cmd):
        target_data = drone.get_target_telemetry(
            target_id=active_target_id(),
            max_age_s=TELEMETRY_MAX_AGE_S,
        )
        telemetry_follow = (
            initial_lock
            and state == State.FOLLOW
            and is_usable_target_telemetry(target_data)
        )
        alt_target_abs = curr_abs_alt if hold_abs_alt is None else hold_abs_alt
        alt_feedforward_vd = 0.0

        if protocol_cmd == "HOLD" or state == State.HOLD:
            vn_cmd, ve_cmd = 0.0, 0.0
            yaw_deg_cmd = curr_yaw
        elif telemetry_follow and state != State.LAND:
            alt_target_abs = target_data["abs_alt"]
            alt_feedforward_vd = target_data["vd"]

            # Use OpenCV's visual commands for horizontal movement and yaw.
            target_vn = target_data["vn"]
            target_ve = target_data["ve"]
            vn_cmd += target_vn * 1.0
            ve_cmd += target_ve * 1.0

            # Clamp horizontal velocity to MAX_FWD
            v_mag = math.hypot(vn_cmd, ve_cmd)
            if v_mag > MAX_FWD:
                vn_cmd = (vn_cmd / v_mag) * MAX_FWD
                ve_cmd = (ve_cmd / v_mag) * MAX_FWD

            # Stateful collision overrides
            if collision_state == "CRITICAL_AVOID":
                rad_heading = math.radians(curr_yaw)
                vn_cmd = -1.5 * math.cos(rad_heading)
                ve_cmd = -1.5 * math.sin(rad_heading)
            elif collision_state == "COLLISION_AVOID":
                vn_cmd *= 0.2
                ve_cmd *= 0.2

        err_alt = alt_target_abs - curr_abs_alt
        vert_spd_cmd = alt_feedforward_vd - 1.5 * err_alt
        vert_spd_cmd = float(np.clip(vert_spd_cmd, -MAX_VERT, MAX_VERT))

        await drone.set_velocity(vn_cmd, ve_cmd, vert_spd_cmd, yaw_deg=yaw_deg_cmd)

    # ── TAKEOFF ────────────────────────────────────────────────────
    # Ground reference from LOCAL NED (no GPS fix in this world, so position()
    # never yields). -down_m is ~0 on the ground; takeoff altitude is a fixed
    # relative climb that matches the leader's hover so the forward camera keeps
    # the LED in frame for the visual lock.
    ground_abs_alt = 0.0
    async for pv in drone._sys.telemetry.position_velocity_ned():
        ground_abs_alt = -pv.position.down_m
        break
    takeoff_alt = MIN_COMMAND_TAKEOFF_ALT
    _log(f"Ground local altitude: {ground_abs_alt:.2f}m. Target relative takeoff: {takeoff_alt:.2f}m")

    # Start telemetry tracking task
    telemetry_task = asyncio.create_task(track_telemetry_loop())

    # Wait until the first NED telemetry sample has populated the altitude.
    while curr_abs_alt is None:
        await asyncio.sleep(0.1)

    # Disable battery failsafes in SITL for followers as well as the leader.
    for name, val in [
        ("COM_LOW_BAT_ACT", 0),
        ("SYS_HAS_MAG", 0),
        ("COM_ARM_MAG_STR", 0),
    ]:
        try:
            await drone._sys.param.set_param_int(name, val)
        except Exception as exc:
            _log(f"Param {name} not set: {exc}")
    for name, val in [("BAT_LOW_THR", 0.02), ("BAT_CRIT_THR", 0.01), ("BAT_EMERGEN_THR", 0.005)]:
        try:
            await drone._sys.param.set_param_float(name, val)
        except Exception as exc:
            _log(f"Param {name} not set: {exc}")

    drone.led_off()
    _log("Arming …")
    await drone.arm()

    # ── Offboard takeoff ───────────────────────────────────────────
    # AUTO takeoff (drone.takeoff) needs a global-position fix this world's x500
    # never gets, so it won't climb. Climb with OFFBOARD POSITION setpoints in
    # LOCAL NED: position offboard lifts reliably from the ground (velocity-only
    # offboard does not engage thrust from a standstill). Once airborne the follow
    # loop switches to velocity offboard, which works fine in flight.
    _log(f"Offboard takeoff to relative altitude {takeoff_alt:.2f} m …")
    await drone.start_offboard()

    # Anchor horizontal position; climb by lowering the NED-down target.
    anchor_n, anchor_e = curr_n, curr_e
    target_d = curr_d - takeoff_alt
    required_takeoff_alt = max(
        MIN_TAKEOFF_REL_ALT,
        min(takeoff_alt - 0.5, takeoff_alt * 0.75),
    )
    deadline = time.time() + TAKEOFF_TIMEOUT_S
    while not shutdown.is_set() and time.time() < deadline:
        rel_ned_alt = -curr_d
        if rel_ned_alt >= required_takeoff_alt:
            airborne = True
            _log(f"Takeoff confirmed: rel_ned={rel_ned_alt:.2f}m")
            break
        # Hold horizontal, command target altitude. Keep streaming so offboard stays engaged.
        await drone.go_to(anchor_n, anchor_e, target_d, yaw_deg=curr_yaw)
        await asyncio.sleep(0.1)

    if not airborne:
        _log(
            f"TAKEOFF_FAILED: rel_ned={-curr_d:.2f}m "
            f"required={required_takeoff_alt:.2f}m"
        )
        pitch_task.cancel()
        telemetry_task.cancel()
        try:
            await drone.stop_offboard()
            await drone.land()
            await asyncio.sleep(3)
            await drone.disarm()
        except Exception as exc:
            _log(f"Takeoff failure cleanup error: {exc}")
        return

    # Stabilise at altitude by holding the takeoff position setpoint.
    stabilize_until = time.time() + TAKEOFF_WAIT_S
    while time.time() < stabilize_until and not shutdown.is_set():
        await drone.go_to(anchor_n, anchor_e, target_d, yaw_deg=curr_yaw)
        await asyncio.sleep(0.1)
    hold_abs_alt = curr_abs_alt
    _log(f"Altitude hold target: abs_alt={hold_abs_alt:.2f}m")

    drone.led_off()       # enable chain only after a stable visual lock
    _log("Offboard active. LED waits for initial lock. State → FOLLOW")
    state = State.FOLLOW
    last_det_t = time.time()


    # ── MAIN LOOP ──────────────────────────────────────────────────
    try:
        while not shutdown.is_set() and state != State.LAND:
            # Publish local telemetry
            rel_abs_alt = curr_abs_alt - ground_abs_alt
            airborne = rel_abs_alt >= MIN_TAKEOFF_REL_ALT and -curr_d >= 1.0
            drone.publish_telemetry(
                curr_n,
                curr_e,
                curr_d,
                curr_vn,
                curr_ve,
                curr_vd,
                curr_yaw,
                curr_abs_alt,
                airborne=airborne,
                target_id=active_target_id(),
            )
            frame = drone.camera_frame()

            if frame is not None:
                # Save frame to disk every second
                now = time.time()
                if now - last_img_t > 1.0:
                    cv2.imwrite(f"camera_drone_{drone_id}.jpg", frame)
                    last_img_t = now

                det = detect_led(frame, state)

                # Apply centroid tracking gate to filter out sudden jumps (background clutter/noise)
                if det is not None:
                    cx_d, cy_d, w_d, h_d, area_d = det
                    now_t = time.time()
                    if last_cx is not None and now_t - last_det_time < 0.5:
                        dist = math.hypot(cx_d - last_cx, cy_d - last_cy)
                        if dist > 150.0:  # reject jumps larger than 150 pixels in 0.5s
                            det = None

                if det is not None and not is_tracking_candidate(det):
                    det = None

                if not initial_lock:
                    if det is not None and is_initial_lock_candidate(det):
                        initial_lock_hits += 1
                        if initial_lock_hits >= INITIAL_LOCK_FRAMES:
                            initial_lock = True
                            det_history.clear()
                            drone.led_on()
                            _log(
                                f"Initial LED lock acquired; enabling chain LED "
                                f"(target_id={active_target_id()})"
                            )
                        else:
                            det = None
                    else:
                        initial_lock_hits = 0
                        det = None

                # Update tracking gate state if we have a valid detection
                if det is not None:
                    cx_d, cy_d, w_d, h_d, area_d = det
                    last_cx, last_cy = cx_d, cy_d
                    last_det_time = time.time()

                det_history.append(1 if det is not None else 0)

                # Decode protocol based on recent detection history
                protocol_cmd = "LOST"
                if len(det_history) == det_history.maxlen:
                    duty = sum(det_history) / len(det_history)
                    edges = sum(1 for i in range(1, len(det_history)) if det_history[i] != det_history[i-1])
                    
                    if duty > 0.8:
                        protocol_cmd = "FOLLOW"
                    elif 0.35 <= duty <= 0.65 and (2 <= edges <= 5):
                        protocol_cmd = "HOLD"
                    elif duty <= 0.2:
                        protocol_cmd = "LOST"
                    else:
                        # Fallback/sticky logic for transitions
                        if state == State.HOLD and duty > 0.2:
                            protocol_cmd = "HOLD"
                        else:
                            protocol_cmd = "FOLLOW"
                elif det is not None:
                    protocol_cmd = "FOLLOW" # Default before history fills up

                if det is not None:
                    # ---- LED detected in this frame (update tracking info) ----
                    cx, cy, w, h, area = det
                    last_det_t = time.time()
                    last_dir = 1 if cx > CX else -1


                    if protocol_cmd == "HOLD":
                        if state != State.HOLD:
                            _log("Blinking detected → HOLD")
                            state = State.HOLD
                        heading = await drone.heading()
                        await send_velocity(0, 0, 0, heading)
                        prev_fwd_speed = 0.0

                        # periodic debug
                        now = time.time()
                        if now - last_log_t > LOG_INTERVAL:
                            last_log_t = now
                            _log(f"HOLD  cx={cx} cy={cy} w={w}")
                    else:
                        if state not in (State.FOLLOW, State.TAKEOFF):
                            _log("Solid LED → FOLLOW")
                            state = State.FOLLOW

                        # ── Stateful Hysteresis Collision Update ───────────────────
                        collision_state = update_collision_state(collision_state, w)

                        # ── Stagger Shift ──────────────────────────────────────────
                        stagger_offset = -20 if (drone_id % 2 == 1) else 20  # tighter formation
                        CX_target = CX + stagger_offset
                        err_x = (cx - CX_target) / CX

                        # ── Pitch-Tilt Camera Compensation ─────────────────────────
                        pitch_comp = (current_pitch_deg / 45.0) * FRAME_H
                        pitch_comp = float(np.clip(pitch_comp, -120.0, 120.0))
                        err_y = (cy - pitch_comp - CY) / CY

                        # ── Distance error based on area under stagger ──────────────
                        err_d = (TARGET_AREA - area) / TARGET_AREA

                        # ── PD Controller & Rate limiting ──────────────────────────
                        err_d_diff = update_pd_filter(err_d, prev_err_d, prev_err_d_diff, ALPHA_D, CTRL_DT)
                        err_x_diff = update_pd_filter(err_x, prev_err_x, prev_err_x_diff, ALPHA_D, CTRL_DT)

                        # Speed outputs
                        fwd_speed = KP_FORWARD * err_d + KD_FORWARD * err_d_diff
                        yaw_adj   = KP_YAW * err_x + KD_YAW * err_x_diff
                        vert_spd  = float(np.clip(KP_ALT * err_y, -MAX_VERT, MAX_VERT))

                        # Save errors for next PD cycle
                        prev_err_d = err_d
                        prev_err_d_diff = err_d_diff
                        prev_err_x = err_x
                        prev_err_x_diff = err_x_diff

                        # ── Obstacle Avoidance Proxy ───────────────────────────────
                        led_bbox = (cx - w // 2, cy - h // 2, w, h)
                        obs_density = compute_obstacle_density(frame, led_bbox=led_bbox, box_size=160, pitch_comp=pitch_comp)
                        
                        lat_speed = 0.0
                        avoiding = False
                        if obs_density > 0.08:
                            avoid_dir = obstacle_avoid_direction(frame, box_size=160)
                            lat_speed = 0.6 * avoid_dir
                            # Scale down forward speed proportionally
                            density_factor = np.clip((obs_density - 0.08) / 0.06, 0.0, 1.0)
                            fwd_speed = fwd_speed * (1.0 - 0.6 * density_factor)
                            avoiding = True

                        # ── Tilt Speed Guard ───────────────────────────────────────
                        if abs(err_y) > 0.6:
                            fwd_speed = fwd_speed * 0.5

                        # ── Apply stateful speed overrides ─────────────────────────
                        if collision_state == "CRITICAL_AVOID":
                            fwd_speed = -1.5  # Bypass rate-limiter for immediate safety backing
                        elif collision_state == "COLLISION_AVOID":
                            fwd_speed = min(fwd_speed, 0.3)
                            fwd_speed = _rate_limit(fwd_speed, prev_fwd_speed, max_accel=2.0, dt=CTRL_DT)
                        else:
                            fwd_speed = _rate_limit(fwd_speed, prev_fwd_speed, max_accel=2.0, dt=CTRL_DT)

                        fwd_speed = float(np.clip(fwd_speed, -MAX_FWD, MAX_FWD))
                        prev_fwd_speed = fwd_speed

                        # ── Translate to NED ───────────────────────────────────────
                        heading = await drone.heading()
                        tgt_hdg = (heading + yaw_adj) % 360.0

                        rad = math.radians(heading)
                        vn = fwd_speed * math.cos(rad) - lat_speed * math.sin(rad)
                        ve = fwd_speed * math.sin(rad) + lat_speed * math.cos(rad)

                        # periodic debug
                        now = time.time()
                        if now - last_log_t > LOG_INTERVAL:
                            last_log_t = now
                            state_label = "AVOID" if avoiding else (collision_state if collision_state != "NORMAL" else "FOLLOW")
                            _log(f"{state_label}  cx={cx} cy={cy} w={w} area={area:.1f} "
                                 f"pitch={current_pitch_deg:+.1f} pitch_comp={pitch_comp:+.1f} "
                                 f"err_x={err_x:+.2f} err_d={err_d:+.2f} "
                                 f"fwd={fwd_speed:+.1f} lat={lat_speed:+.1f} yaw_adj={yaw_adj:+.1f}")

                        await send_velocity(vn, ve, vert_spd, tgt_hdg)
                else:
                    # ---- LED lost in this frame ----
                    dt_lost = time.time() - last_det_t
                    heading = await drone.heading()
                    prev_fwd_speed = 0.0

                    if protocol_cmd == "HOLD":
                        # Maintain position even if currently lost in the blink cycle
                        if state != State.HOLD:
                            _log("Blinking detected (off cycle) → HOLD")
                            state = State.HOLD
                        await send_velocity(0, 0, 0, heading)
                        last_det_t = time.time() # Reset timeout during HOLD mode
                    else:
                        # Check obstacle density even if LED is lost
                        pitch_comp = (current_pitch_deg / 45.0) * FRAME_H
                        pitch_comp = float(np.clip(pitch_comp, -120.0, 120.0))
                        obs_density = compute_obstacle_density(frame, led_bbox=None, box_size=160, pitch_comp=pitch_comp)
                        lat_speed = 0.0
                        if obs_density > 0.08:
                            avoid_dir = obstacle_avoid_direction(frame, box_size=160)
                            lat_speed = 0.6 * avoid_dir

                        target_bearing = active_target_bearing()
                        if target_bearing is not None:
                            yaw_error = angle_diff_deg(target_bearing, heading)
                            last_dir = 1 if yaw_error >= 0 else -1
                            search_hdg = step_heading_towards(
                                heading,
                                target_bearing,
                                TARGET_SEARCH_RATE * CTRL_DT,
                            )
                        else:
                            search_hdg = (heading
                                          + last_dir * SEARCH_RATE * CTRL_DT
                                          ) % 360.0

                        # periodic debug for lost state
                        now = time.time()
                        if now - last_log_t > LOG_INTERVAL:
                            last_log_t = now
                            lock_label = "WAIT_INITIAL" if not initial_lock else "LOST"
                            bearing_label = (
                                f"{target_bearing:.1f}"
                                if target_bearing is not None
                                else "blind"
                            )
                            _log(
                                f"{lock_label}  dt={dt_lost:.1f}s  state={state.value} "
                                f"target={active_target_id()} bearing={bearing_label} "
                                f"obs_density={obs_density:.3f}"
                            )

                        if abs(lat_speed) > 0:
                            rad = math.radians(heading)
                            vn = -lat_speed * math.sin(rad)
                            ve = lat_speed * math.cos(rad)
                        else:
                            vn, ve = 0.0, 0.0

                        if not initial_lock:
                            # Startup mode: actively rotate until the first stable LED lock.
                            if dt_lost < INITIAL_SEARCH_TIMEOUT:
                                if state != State.YAW_SEARCH:
                                    _log("Searching for initial LED → YAW_SEARCH")
                                    state = State.YAW_SEARCH
                                await send_velocity(vn, ve, 0, search_hdg)
                            else:
                                _log("Initial connection timeout → LAND")
                                state = State.LAND
                        else:
                            # Normal tracking lost mode: turn immediately and reacquire.
                            if dt_lost < LAND_TIMEOUT:
                                if state != State.YAW_SEARCH:
                                    _log("LED lost → YAW_SEARCH")
                                    state = State.YAW_SEARCH
                                await send_velocity(vn, ve, 0, search_hdg)
                            else:
                                _log("Timeout → LAND")
                                state = State.LAND

                update_camera_status(det)
            else:
                update_camera_status(None)

            await asyncio.sleep(CTRL_DT)

    except asyncio.CancelledError:
        pass
    except Exception as exc:
        _log(f"ERROR: {exc}")

    finally:
        pitch_task.cancel()
        telemetry_task.cancel()

    # ── LANDING ────────────────────────────────────────────────────
    _log("Landing …")
    try:
        heading = await drone.heading()
        await drone.set_velocity(0, 0, 0, yaw_deg=heading)
        await asyncio.sleep(0.5)
        await drone.stop_offboard()
        await drone.land()
        drone.led_off()
        await asyncio.sleep(15)
        await drone.disarm()
        _log("Landed & disarmed ✓")
    except Exception as exc:
        _log(f"Landing error: {exc}")


# ══════════════════════════════════════════════════════════════════════
#  Main entry point
# ══════════════════════════════════════════════════════════════════════

async def camera_preview_loop(drones, shutdown, camera_status):
    if not SHOW_CAMERA:
        return

    seen_frames = set()
    while not shutdown.is_set():
        try:
            for drone in drones:
                frame = drone.camera_frame()
                if frame is None:
                    continue
                if drone.drone_id not in seen_frames:
                    seen_frames.add(drone.drone_id)
                    print(f"Camera preview live for drone {drone.drone_id}")

                status = camera_status.get(drone.drone_id, {})
                key = show_camera_frame(
                    drone.drone_id,
                    frame,
                    status.get("state", State.TAKEOFF),
                    status.get("det"),
                    status.get("initial_lock", False),
                    status.get("protocol_cmd", "BOOT"),
                    status.get("target_id", 0 if drone.drone_id == 1 else drone.drone_id - 1),
                    status.get("yaw", 0.0),
                    status.get("pitch", 0.0),
                )
                if key in (27, ord("q")):
                    print("Camera preview requested shutdown")
                    shutdown.set()
                    break
        except cv2.error as exc:
            print(f"Camera preview disabled: {exc}")
            break

        await asyncio.sleep(0.03)


async def main():
    rclpy.init()

    stop_thread = threading.Event()
    stop_async  = asyncio.Event()

    def _on_sig(*_):
        stop_thread.set()
        stop_async.set()
    signal.signal(signal.SIGINT, _on_sig)

    # ── create & connect followers ────────────────────────────────
    all_drones = [Drone(drone_id=did) for did in FOLLOWER_IDS]
    connected_drones = []

    print("Connecting follower drones …")
    for d in all_drones:
        try:
            await d.connect(wait_health=False)
            print(f"  Drone {d.drone_id}: connected ✓")
            connected_drones.append(d)
        except Exception as exc:
            print(f"  Drone {d.drone_id}: FAILED — {exc} (retrying once)")
            try:
                await d.connect(wait_health=False)
                print(f"  Drone {d.drone_id}: connected on retry ✓")
                connected_drones.append(d)
            except Exception as exc2:
                print(f"  Drone {d.drone_id}: FAILED permanently — {exc2}")

    if not connected_drones:
        print("No drones connected successfully. Exiting.")
        rclpy.shutdown()
        return

    # ── camera-spin thread ────────────────────────────────────────
    cam = threading.Thread(target=_camera_spin,
                           args=(connected_drones, stop_thread), daemon=True)
    cam.start()
    await asyncio.sleep(4.0)       # wait for ROS bridges (longer for slow SITL)
    camera_status = {
        d.drone_id: {
            "state": State.TAKEOFF,
            "det": None,
            "initial_lock": False,
            "protocol_cmd": "BOOT",
            "target_id": 0 if d.drone_id == 1 else d.drone_id - 1,
            "yaw": 0.0,
            "pitch": 0.0,
        }
        for d in connected_drones
    }
    preview_task = asyncio.create_task(
        camera_preview_loop(connected_drones, stop_async, camera_status)
    )

    # ── launch follower coroutines ────────────────────────────────
    tasks = [
        asyncio.create_task(
            follower(
                d,
                d.drone_id,
                stop_async,
                start_delay=float(i * FOLLOWER_START_STAGGER_S),
                camera_status=camera_status,
            )
        )
        for i, d in enumerate(connected_drones)
    ]
    connected_ids = [d.drone_id for d in connected_drones]
    print(f"Follower missions launched for: {connected_ids}")

    results = await asyncio.gather(*tasks, return_exceptions=True)
    for drone_id, result in zip(connected_ids, results):
        if isinstance(result, Exception):
            print(f"Follower mission for drone {drone_id} failed: {result!r}")

    # ── cleanup ───────────────────────────────────────────────────
    preview_task.cancel()
    try:
        await preview_task
    except asyncio.CancelledError:
        pass
    stop_thread.set()
    cam.join(timeout=5)
    if SHOW_CAMERA:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
    for d in connected_drones:
        await d.close()
    rclpy.shutdown()
    print("All followers done.")



if __name__ == "__main__":
    asyncio.run(main())
