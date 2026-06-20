#!/usr/bin/env python3
import sys
import numpy as np
import cv2

# Pure functions to be tested and shared with follower_mission.py

def update_collision_state(state, w):
    COLLISION_W_ENTER = 23.0
    COLLISION_W_EXIT  = 20.0
    CRITICAL_W_ENTER  = 28.0
    CRITICAL_W_EXIT   = 25.0

    if w > CRITICAL_W_ENTER:
        state = "CRITICAL_AVOID"
    elif state == "CRITICAL_AVOID" and w < CRITICAL_W_EXIT:
        state = "COLLISION_AVOID"
    elif state != "CRITICAL_AVOID" and w > COLLISION_W_ENTER:
        state = "COLLISION_AVOID"
    elif state == "COLLISION_AVOID" and w < COLLISION_W_EXIT:
        state = "NORMAL"
    return state


def _rate_limit(target, prev, max_accel, dt):
    max_step = max_accel * dt
    return float(np.clip(target, prev - max_step, prev + max_step))


def update_pd_filter(err, prev_err, prev_err_diff, alpha, dt):
    raw_diff = (err - prev_err) / dt
    smoothed_diff = alpha * raw_diff + (1 - alpha) * prev_err_diff
    return smoothed_diff


def angle_diff_deg(target_deg, current_deg):
    return (target_deg - current_deg + 540.0) % 360.0 - 180.0


def step_heading_towards(current_deg, target_deg, max_step_deg):
    err = angle_diff_deg(target_deg, current_deg)
    step = float(np.clip(err, -max_step_deg, max_step_deg))
    return (current_deg + step) % 360.0


def is_tracking_candidate(det):
    if det is None:
        return False
    _cx, _cy, w, h, area = det
    return area >= 20.0 and w >= 6 and h >= 5


def is_initial_lock_candidate(det):
    if det is None:
        return False
    _cx, _cy, w, h, area = det
    return area >= 45.0 and w >= 8 and h >= 6


def is_valid_contour(area, bw, bh, max_aspect, solidity=None):
    MIN_AREA = 8.0
    MAX_ALLOWED_AREA = 950.0
    MIN_BLOB_DIM = 3.0
    MAX_BLOB_DIM = 48
    SOLIDITY_MIN_AREA = 50.0
    SOLIDITY_THRESHOLD = 0.82

    if area < MIN_AREA or area > MAX_ALLOWED_AREA:
        return False
    if bw < MIN_BLOB_DIM or bh < MIN_BLOB_DIM:
        return False
    if bw > MAX_BLOB_DIM or bh > MAX_BLOB_DIM:
        return False
    aspect = bw / max(bh, 1.0)
    if aspect > max_aspect:
        return False
    if area >= SOLIDITY_MIN_AREA:
        if solidity is not None and solidity < SOLIDITY_THRESHOLD:
            return False
    return True




def compute_obstacle_density(frame, led_bbox=None, box_size=160):
    FRAME_W, FRAME_H = 640, 480
    cx0, cy0 = FRAME_W // 2 - box_size // 2, FRAME_H // 2 - box_size // 2
    roi = frame[cy0:cy0+box_size, cx0:cx0+box_size].copy()

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

    edges = cv2.Canny(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), 50, 150)
    return float(np.count_nonzero(edges) / edges.size)


# Test cases

def test_collision_hysteresis():
    state = "NORMAL"
    
    # Normal behavior
    state = update_collision_state(state, 15.0)
    assert state == "NORMAL", f"Expected NORMAL, got {state}"
    
    # Enter Collision
    state = update_collision_state(state, 24.0)
    assert state == "COLLISION_AVOID", f"Expected COLLISION_AVOID, got {state}"
    
    # Hysteresis keep collision (above exit 20.0)
    state = update_collision_state(state, 22.0)
    assert state == "COLLISION_AVOID", f"Expected COLLISION_AVOID, got {state}"
    
    # Exit collision
    state = update_collision_state(state, 19.0)
    assert state == "NORMAL", f"Expected NORMAL, got {state}"
    
    # Direct jump to Critical Avoid from Normal (no intermediate frame)
    state = update_collision_state(state, 30.0)
    assert state == "CRITICAL_AVOID", f"Expected CRITICAL_AVOID, got {state}"
    
    # Hysteresis keep critical (above exit 25.0)
    state = update_collision_state(state, 26.0)
    assert state == "CRITICAL_AVOID", f"Expected CRITICAL_AVOID, got {state}"
    
    # Exit critical to collision (below 25.0 but above 20.0)
    state = update_collision_state(state, 24.0)
    assert state == "COLLISION_AVOID", f"Expected COLLISION_AVOID, got {state}"
    
    # Exit collision to normal (below 20.0)
    state = update_collision_state(state, 18.0)
    assert state == "NORMAL", f"Expected NORMAL, got {state}"
    print("test_collision_hysteresis passed ✓")


def test_pd_controller_smoothing():
    # Rate limit test
    prev = 1.0
    res = _rate_limit(2.0, prev, max_accel=3.0, dt=0.05)
    # max step = 3 * 0.05 = 0.15. So 2.0 should be clamped to 1.15
    assert abs(res - 1.15) < 1e-5, f"Expected 1.15, got {res}"

    # Smooth filter test
    alpha = 0.3
    dt = 0.05
    prev_err = 0.0
    prev_err_diff = 0.0
    
    # Error changes by 1.0 in 0.05s (raw diff = 20.0)
    smoothed = update_pd_filter(1.0, prev_err, prev_err_diff, alpha, dt)
    # 0.3 * 20.0 + 0.7 * 0 = 6.0
    assert abs(smoothed - 6.0) < 1e-5, f"Expected 6.0, got {smoothed}"
    print("test_pd_controller_smoothing passed ✓")


def test_solidity_filter_small_contours():
    # Small area: solidity should be bypassed
    assert is_valid_contour(area=20.0, bw=5, bh=5, max_aspect=4.0, solidity=0.5) is True
    
    # Large area with low solidity: should be rejected
    assert is_valid_contour(area=60.0, bw=10, bh=10, max_aspect=4.0, solidity=0.5) is False
    
    # Large area with high solidity: should be accepted
    assert is_valid_contour(area=60.0, bw=10, bh=10, max_aspect=4.0, solidity=0.9) is True
    
    # Medium-large area (e.g. 600px at close distance): should be accepted if solidity is high
    assert is_valid_contour(area=600.0, bw=28, bh=28, max_aspect=4.0, solidity=0.9) is True

    # Bounding box width up to 35 (below 48 limit): should be accepted if solidity is high
    assert is_valid_contour(area=60.0, bw=35, bh=10, max_aspect=4.0, solidity=0.9) is True

    
    # Overly large area (>950): should be rejected
    assert is_valid_contour(area=1000.0, bw=10, bh=10, max_aspect=4.0, solidity=0.9) is False
    
    # Overly large bounding box dimensions (>48): should be rejected
    assert is_valid_contour(area=60.0, bw=55, bh=10, max_aspect=4.0, solidity=0.9) is False
    print("test_solidity_filter_small_contours passed ✓")


def test_initial_lock_rejects_tiny_blob():
    tiny_blob = (228, 243, 5, 5, 16.0)
    normal_led = (253, 236, 15, 10, 91.5)

    assert is_tracking_candidate(tiny_blob) is False
    assert is_initial_lock_candidate(tiny_blob) is False
    assert is_tracking_candidate(normal_led) is True
    assert is_initial_lock_candidate(normal_led) is True
    print("test_initial_lock_rejects_tiny_blob passed ✓")


def test_heading_step_wraparound():
    assert angle_diff_deg(5.0, 355.0) == 10.0
    assert angle_diff_deg(355.0, 5.0) == -10.0

    assert step_heading_towards(355.0, 5.0, 3.0) == 358.0
    assert step_heading_towards(5.0, 355.0, 3.0) == 2.0
    assert step_heading_towards(10.0, 14.0, 10.0) == 14.0
    print("test_heading_step_wraparound passed ✓")




def test_obstacle_density_excludes_target_bbox():
    # Create a 640x480 black image
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Draw some "obstacle" texture inside the central region (which is CX=320, CY=240, ROI is [160..480] on X, [240-80..240+80] = [160..320] on Y)
    # Let's draw high-contrast white circles/lines inside the central ROI
    cv2.circle(frame, (320, 240), 5, (255, 255, 255), -1)
    
    # Density without masking should be non-zero
    d1 = compute_obstacle_density(frame, led_bbox=None, box_size=160)
    assert d1 > 0, "Expected non-zero edge density"
    
    # Masking out the center circular area (e.g. led_bbox = (315, 235, 10, 10))
    # It should mask the region around (320, 240) and make the density 0.
    d2 = compute_obstacle_density(frame, led_bbox=(315, 235, 10, 10), box_size=160)
    assert d2 == 0, f"Expected zero density after masking, got {d2}"
    print("test_obstacle_density_excludes_target_bbox passed ✓")


def run_all_tests():
    failures = []
    for test in [
        test_collision_hysteresis,
        test_pd_controller_smoothing,
        test_solidity_filter_small_contours,
        test_initial_lock_rejects_tiny_blob,
        test_heading_step_wraparound,
        test_obstacle_density_excludes_target_bbox,
    ]:
        try:
            test()
        except AssertionError as e:
            failures.append((test.__name__, str(e)))
        except Exception as e:
            failures.append((test.__name__, f"Unexpected error: {str(e)}"))
    return failures


if __name__ == "__main__":
    failures = run_all_tests()
    if failures:
        print(f"FAILED TESTS: {failures}")
        sys.exit(1)
    print("All tests passed ✓")
    sys.exit(0)
