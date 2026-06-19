#!/usr/bin/env python3
"""
Follower mission — vision-based leader tracking for the GENERA hackathon.

Each follower drone uses its front-facing camera to detect the green LED
beacon on the drone ahead and follows it with a P-controller.

State machine:
    TAKEOFF → FOLLOW ⇄ SAFE_HOVER → YAW_SEARCH → LAND

Usage:
    source /opt/ros/humble/setup.bash
    python3 examples/follower_mission.py
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
TAKEOFF_WAIT_S = 12      # stabilisation time after takeoff

# Camera
FRAME_W, FRAME_H = 640, 480
CX, CY = FRAME_W / 2, FRAME_H / 2      # frame centre

# HSV thresholds for the green LED
# Narrow hue window + high saturation to reject grass/ground
HSV_LO = np.array([40, 150, 150])
HSV_HI = np.array([80, 255, 255])

# Minimum contour area (pixels²) — reject noise below this
MIN_AREA = 8.0

# Blob shape filters
MIN_BLOB_DIM  = 3     # pixels — reject contours thinner than this
MAX_ASPECT    = 4.0   # w/h ratio — reject long horizontal strips (grass edges)

# Target LED bounding-box width (px) at desired following distance (~3-4 m)
TARGET_W = 18.0

# ── P-controller gains ─────────────────────────────────────────────
KP_YAW     = 25.0      # deg heading adjustment per normalised x-error
KP_FORWARD = 2.5       # m/s per normalised width-error
KP_ALT     = 1.0       # m/s per normalised y-error

# ── Speed clamps ───────────────────────────────────────────────────
MAX_FWD  = 3.0   # m/s
MAX_VERT = 1.0   # m/s

# ── Timing ─────────────────────────────────────────────────────────
HOVER_TIMEOUT  = 3.0    # seconds of no detection before yaw-search
SEARCH_RATE    = 8.0    # deg/s yaw rotation in search mode (reduced to avoid motion blur)
LAND_TIMEOUT   = 20.0   # seconds before auto-land
CTRL_DT        = 0.05   # 20 Hz control loop
LOG_INTERVAL   = 2.0    # seconds between periodic debug prints


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
#  CV detector
# ══════════════════════════════════════════════════════════════════════

_morph_kernel = np.ones((3, 3), np.uint8)

def detect_led(frame, state=None):
    """Return (cx, cy, w, h, area) of the largest green blob, or None."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Adaptive thresholds during active search to handle motion blur smearing
    if state == State.YAW_SEARCH:
        lo = np.array([40, 100, 100])  # loosen saturation/value
        hi = np.array([80, 255, 255])
        max_aspect = 8.0               # allow horizontally elongated blobs due to motion blur
    else:
        lo = HSV_LO
        hi = HSV_HI
        max_aspect = MAX_ASPECT
        
    mask = cv2.inRange(hsv, lo, hi)

    # morphological cleanup
    mask = cv2.erode(mask, _morph_kernel, iterations=1)
    mask = cv2.dilate(mask, _morph_kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0.0
    for c in contours:
        a = cv2.contourArea(c)
        if a < MIN_AREA:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        # reject tiny blobs
        if bw < MIN_BLOB_DIM or bh < MIN_BLOB_DIM:
            continue
        # reject elongated horizontal strips (grass edges)
        aspect = bw / max(bh, 1)
        if aspect > max_aspect:
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

async def follower(drone, drone_id, shutdown):
    state = State.TAKEOFF
    last_det_t = time.time()
    last_dir   = 1          # +1 = LED was to the right, −1 = left
    last_log_t = 0.0        # timestamp of last periodic debug print

    # For protocol decoding
    det_history = collections.deque(maxlen=int(2.0 / CTRL_DT))  # 2 seconds history

    # For tracking gate
    last_cx, last_cy = None, None
    last_det_time = 0.0

    _log = lambda msg: print(f"[Drone {drone_id}] {msg}")

    # ── TAKEOFF ────────────────────────────────────────────────────
    # Get starting ground absolute altitude to compute relative takeoff altitude
    ground_abs_alt = None
    async for pos in drone._sys.telemetry.position():
        ground_abs_alt = pos.absolute_altitude_m
        break
    takeoff_alt = TARGET_ABS_ALT - ground_abs_alt
    _log(f"Ground absolute altitude: {ground_abs_alt:.2f}m. Target relative takeoff: {takeoff_alt:.2f}m")

    _log("Arming …")
    await drone.arm()
    _log(f"Taking off to relative altitude {takeoff_alt:.2f} m …")
    await drone.takeoff(altitude_m=takeoff_alt)
    await asyncio.sleep(TAKEOFF_WAIT_S)

    _log("Starting offboard mode …")
    await drone.start_offboard()
    drone.led_on()        # own LED on — enables chain following
    _log("Offboard + LED active.  State → FOLLOW")
    state = State.FOLLOW
    last_det_t = time.time()

    # ── MAIN LOOP ──────────────────────────────────────────────────
    try:
        while not shutdown.is_set() and state != State.LAND:
            frame = drone.camera_frame()

            if frame is not None:
                det = detect_led(frame, state)

                # Apply centroid tracking gate to filter out sudden jumps (background clutter/noise)
                if det is not None:
                    cx_d, cy_d, w_d, h_d, area_d = det
                    now_t = time.time()
                    if last_cx is not None and now_t - last_det_time < 0.5:
                        dist = math.hypot(cx_d - last_cx, cy_d - last_cy)
                        if dist > 150.0:  # reject jumps larger than 150 pixels in 0.5s
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
                        await drone.set_velocity(0, 0, 0, yaw_deg=heading)

                        # periodic debug
                        now = time.time()
                        if now - last_log_t > LOG_INTERVAL:
                            last_log_t = now
                            _log(f"HOLD  cx={cx} cy={cy} w={w}")
                    else:
                        if state not in (State.FOLLOW, State.TAKEOFF):
                            _log("Solid LED → FOLLOW")
                            state = State.FOLLOW

                        # normalised errors (-1 … +1)
                        err_x = (cx - CX) / CX
                        err_y = (cy - CY) / CY
                        err_d = (TARGET_W - w) / TARGET_W   # >0 → too far

                        # controller outputs
                        yaw_adj   = KP_YAW * err_x
                        fwd_speed = float(np.clip(KP_FORWARD * err_d,
                                                  -MAX_FWD, MAX_FWD))
                        vert_spd  = float(np.clip(KP_ALT * err_y,
                                                  -MAX_VERT, MAX_VERT))

                        heading = await drone.heading()
                        tgt_hdg = (heading + yaw_adj) % 360.0

                        # body-forward → NED
                        rad = math.radians(heading)
                        vn = fwd_speed * math.cos(rad)
                        ve = fwd_speed * math.sin(rad)

                        # periodic debug
                        now = time.time()
                        if now - last_log_t > LOG_INTERVAL:
                            last_log_t = now
                            _log(f"FOLLOW  cx={cx} cy={cy} w={w} "
                                 f"err_x={err_x:+.2f} err_d={err_d:+.2f} "
                                 f"fwd={fwd_speed:+.1f} yaw_adj={yaw_adj:+.1f}")

                        await drone.set_velocity(vn, ve, vert_spd,
                                                 yaw_deg=tgt_hdg)
                else:
                    # ---- LED lost in this frame ----
                    dt_lost = time.time() - last_det_t
                    heading = await drone.heading()

                    if protocol_cmd == "HOLD":
                        # If we are in hold, just maintain position even if currently lost in the blink cycle
                        if state != State.HOLD:
                            _log("Blinking detected (off cycle) → HOLD")
                            state = State.HOLD
                        await drone.set_velocity(0, 0, 0, yaw_deg=heading)
                        last_det_t = time.time() # Reset timeout during HOLD mode
                    else:
                        # periodic debug for lost state
                        now = time.time()
                        if now - last_log_t > LOG_INTERVAL:
                            last_log_t = now
                            _log(f"LOST  dt={dt_lost:.1f}s  state={state.value}")

                        if dt_lost < HOVER_TIMEOUT:
                            if state != State.SAFE_HOVER:
                                _log("LED lost → SAFE_HOVER")
                                state = State.SAFE_HOVER
                            await drone.set_velocity(0, 0, 0, yaw_deg=heading)

                        elif dt_lost < LAND_TIMEOUT:
                            if state != State.YAW_SEARCH:
                                _log("Searching → YAW_SEARCH")
                                state = State.YAW_SEARCH
                            search_hdg = (heading
                                          + last_dir * SEARCH_RATE * CTRL_DT
                                          ) % 360.0
                            await drone.set_velocity(0, 0, 0,
                                                     yaw_deg=search_hdg)
                        else:
                            _log("Timeout → LAND")
                            state = State.LAND

            await asyncio.sleep(CTRL_DT)

    except asyncio.CancelledError:
        pass
    except Exception as exc:
        _log(f"ERROR: {exc}")

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

async def main():
    rclpy.init()

    stop_thread = threading.Event()
    stop_async  = asyncio.Event()

    def _on_sig(*_):
        stop_thread.set()
        stop_async.set()
    signal.signal(signal.SIGINT, _on_sig)

    # ── create & connect followers ────────────────────────────────
    drones = [Drone(drone_id=did) for did in FOLLOWER_IDS]

    print("Connecting follower drones …")
    for d in drones:
        try:
            await d.connect()
            print(f"  Drone {d.drone_id}: connected ✓")
        except Exception as exc:
            print(f"  Drone {d.drone_id}: FAILED — {exc}")

    # ── camera-spin thread ────────────────────────────────────────
    cam = threading.Thread(target=_camera_spin,
                           args=(drones, stop_thread), daemon=True)
    cam.start()
    await asyncio.sleep(2.0)       # wait for ROS bridges

    # ── launch follower coroutines ────────────────────────────────
    tasks = [
        asyncio.create_task(follower(d, d.drone_id, stop_async))
        for d in drones
    ]
    print(f"Follower missions launched: {FOLLOWER_IDS}")

    await asyncio.gather(*tasks, return_exceptions=True)

    # ── cleanup ───────────────────────────────────────────────────
    stop_thread.set()
    cam.join(timeout=5)
    for d in drones:
        await d.close()
    rclpy.shutdown()
    print("All followers done.")


if __name__ == "__main__":
    asyncio.run(main())
