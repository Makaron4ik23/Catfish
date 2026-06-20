#!/usr/bin/env python3
"""Sample Gazebo drone poses and evaluate chain formation."""
import math
import re
import subprocess
import sys
import time

DRONE_IDS = [0, 1, 2, 3]
SAMPLE_INTERVAL = 2.0
DURATION = 130.0
LEADER_ID = 0
CHAIN_TOLERANCE_M = 8.0
MIN_ALT_M = 5.0
MIN_CHAIN_SAMPLES = 5


def get_pose(drone_id: int):
    name = f"x500_mono_cam_{drone_id}"
    try:
        out = subprocess.check_output(
            ["gz", "model", "-m", name, "-p"],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=8,
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        return None

    m = re.search(
        r"\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]\s*\n\s*\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]",
        out,
    )
    if not m:
        return None
    x, y, z, roll, pitch, yaw = map(float, m.groups())
    return {"x": x, "y": y, "z": z, "yaw": yaw}


def chain_score(poses, flight_axis=None):
    """Return True if drones form a chain behind leader along flight axis."""
    if any(poses[i] is None for i in DRONE_IDS):
        return False, "missing pose"

    alt_ok = all(poses[i]["z"] > MIN_ALT_M for i in DRONE_IDS)
    if not alt_ok:
        return False, "not airborne"

    leader = poses[LEADER_ID]
    lx, ly = leader["x"], leader["y"]
    if flight_axis is None:
        heading = leader["yaw"]
        # In this PX4/Gazebo setup yaw=0/North maps to Gazebo +Y.
        flight_x = math.sin(heading)
        flight_y = math.cos(heading)
    else:
        flight_x, flight_y = flight_axis
    behind_x = -flight_x
    behind_y = -flight_y

    projections = []
    for i in DRONE_IDS:
        dx = poses[i]["x"] - lx
        dy = poses[i]["y"] - ly
        proj_behind = dx * behind_x + dy * behind_y
        projections.append((i, proj_behind))

    followers = [p for i, p in projections if i != LEADER_ID]
  # followers should be behind leader (positive projection along behind vector)
    if any(p < -2.0 for p in followers):
        return False, f"follower ahead of leader: {projections}"

    # increasing chain: drone 3 furthest behind, drone 1 closest
    f1 = projections[1][1]
    f2 = projections[2][1]
    f3 = projections[3][1]
    ordered = f1 < f2 < f3
    spaced = (f2 - f1) > 1.0 and (f3 - f2) > 1.0
    if ordered and spaced:
        return True, f"chain OK projections(m): {projections}"
    return False, f"not chained: {projections}"


def movement_score(history):
    """Check leader moved significantly during monitoring."""
    leader_hist = [h[LEADER_ID] for h in history if h.get(LEADER_ID)]
    if len(leader_hist) < 3:
        return False, 0.0
    dx = leader_hist[-1]["x"] - leader_hist[0]["x"]
    dy = leader_hist[-1]["y"] - leader_hist[0]["y"]
    dist = (dx * dx + dy * dy) ** 0.5
    return dist > 10.0, dist


def main():
    print(f"Monitoring formation for {DURATION:.0f}s (every {SAMPLE_INTERVAL}s)")
    history = []
    chain_hits = 0
    flight_axis = None
    last_leader_pose = None
    start = time.time()

    while time.time() - start < DURATION:
        poses = {i: get_pose(i) for i in DRONE_IDS}
        leader_pose = poses.get(LEADER_ID)
        if leader_pose and last_leader_pose:
            dx = leader_pose["x"] - last_leader_pose["x"]
            dy = leader_pose["y"] - last_leader_pose["y"]
            dist = math.hypot(dx, dy)
            if dist > 1.0:
                flight_axis = (dx / dist, dy / dist)
        if leader_pose:
            last_leader_pose = leader_pose
        history.append(poses)
        ts = time.time() - start
        line = f"t={ts:5.1f}s"
        for i in DRONE_IDS:
            p = poses[i]
            if p:
                line += f" | d{i}: x={p['x']:.1f} y={p['y']:.1f} z={p['z']:.1f}"
            else:
                line += f" | d{i}: NO_POSE"
        ok, reason = chain_score(poses, flight_axis=flight_axis)
        if ok:
            chain_hits += 1
            line += " | CHAIN"
        else:
            line += f" | {reason}"
        print(line, flush=True)
        time.sleep(SAMPLE_INTERVAL)

    moved, dist = movement_score(history)
    print("\n=== SUMMARY ===")
    print(f"Leader horizontal travel: {dist:.1f} m (moved={moved})")
    print(f"Chain formation samples: {chain_hits}/{len(history)}")

    success = moved and chain_hits >= MIN_CHAIN_SAMPLES
    if success:
        print("RESULT: PASS — followers appear to fly in chain behind leader")
        return 0
    print("RESULT: FAIL — formation or movement criteria not met")
    return 1


if __name__ == "__main__":
    sys.exit(main())
