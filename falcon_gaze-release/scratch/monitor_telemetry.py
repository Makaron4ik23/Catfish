#!/usr/bin/env python3
"""Monitor swarm chain formation via /drone_{id}/telemetry ROS topics."""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DRONE_IDS = [0, 1, 2, 3]
SAMPLE_INTERVAL = 2.0
DURATION = 100.0
MIN_ALT_M = 2.0
MIN_CHAIN_SAMPLES = 4
MAX_TELEMETRY_AGE_S = 3.0


class TelemetryMonitor(Node):
    def __init__(self):
        super().__init__("telemetry_monitor")
        self.data = {i: None for i in DRONE_IDS}
        for i in DRONE_IDS:
            self.create_subscription(
                String,
                f"/drone_{i}/telemetry",
                lambda msg, idx=i: self._cb(idx, msg),
                10,
            )

    def _cb(self, idx, msg):
        try:
            self.data[idx] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass


def chain_ok(poses):
    if any(poses[i] is None for i in DRONE_IDS):
        return False, "missing telemetry"

    now = time.time()
    stale = [
        i for i in DRONE_IDS
        if poses[i].get("t") is not None and now - poses[i]["t"] > MAX_TELEMETRY_AGE_S
    ]
    if stale:
        return False, f"stale telemetry ids={stale}"

    alts = [-poses[i]["d"] for i in DRONE_IDS]
    if any(a < MIN_ALT_M for a in alts):
        return False, f"low altitude alts={alts}"
    if any(not poses[i].get("airborne", alts[i] >= MIN_ALT_M) for i in DRONE_IDS):
        airborne = [poses[i].get("airborne") for i in DRONE_IDS]
        return False, f"not airborne flags={airborne}"

    # Flight axis from leader yaw (degrees)
    yaw = math.radians(poses[0]["yaw"])
    fx, fy = math.cos(yaw), math.sin(yaw)

    projections = []
    for i in DRONE_IDS:
        n, e = poses[i]["n"], poses[i]["e"]
        proj = n * fx + e * fy
        projections.append((i, proj))

    n0 = projections[0][1]
    followers = [projections[i][1] for i in (1, 2, 3)]
    if any(f > n0 + 2.0 for f in followers):
        return False, f"follower ahead of leader: {projections}"

    f1, f2, f3 = followers
    ordered = f1 < f2 < f3
    spaced = (f2 - f1) > 0.5 and (f3 - f2) > 0.5
    gaps = [n0 - f1, f1 - f2, f2 - f3]
    if ordered and spaced:
        return True, f"chain gaps along heading(m): {gaps}"
    return False, f"not chained projections: {projections}"


def main():
    rclpy.init()
    node = TelemetryMonitor()
    chain_hits = 0
    history = []
    start = time.time()

    print(f"Telemetry monitor {DURATION:.0f}s every {SAMPLE_INTERVAL}s")
    while time.time() - start < DURATION:
        spin_until = time.time() + 0.5
        while time.time() < spin_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        poses = dict(node.data)
        history.append(poses)
        ts = time.time() - start
        line = f"t={ts:5.1f}s"
        for i in DRONE_IDS:
            p = poses[i]
            if p:
                alt = -p["d"]
                line += f" | d{i}: N={p['n']:.1f} E={p['e']:.1f} alt={alt:.1f}"
            else:
                line += f" | d{i}: --"
        ok, reason = chain_ok(poses)
        if ok:
            chain_hits += 1
            line += " | CHAIN"
        else:
            line += f" | {reason}"
        print(line, flush=True)
        time.sleep(SAMPLE_INTERVAL)

    leader_hist = [h[0] for h in history if h.get(0)]
    moved = 0.0
    if len(leader_hist) >= 2:
        dn = leader_hist[-1]["n"] - leader_hist[0]["n"]
        de = leader_hist[-1]["e"] - leader_hist[0]["e"]
        moved = math.hypot(dn, de)

    print("\n=== SUMMARY ===")
    print(f"Leader NED travel: {moved:.1f} m")
    print(f"Chain samples: {chain_hits}/{len(history)}")
    success = moved > 8.0 and chain_hits >= MIN_CHAIN_SAMPLES
    print("RESULT: PASS" if success else "RESULT: FAIL")
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
