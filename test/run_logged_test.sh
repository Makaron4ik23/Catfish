#!/bin/bash
#
# Launch logger, main drone, and helper drones.
#
# Logger reads positions directly from Gazebo via:
#   gz topic -e -t /world/baylands_custom/pose/info
# No Gazebo→ROS bridge needed.
#
# LED states come directly from Drone SDK's ROS2 publishers.
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   bash test/run_logged_test.sh [--drones 4]
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NUM_DRONES=4

while [[ $# -gt 0 ]]; do
    case "$1" in
        --drones) NUM_DRONES="$2"; shift 2 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

cleanup() {
    echo ""
    echo "Shutting down..."
    kill $MAIN_PID $HELPER_PID $LOGGER_PID 2>/dev/null || true
    pkill -f image_bridge
    pkill -f parameter_bridge
    wait 2>/dev/null || true
    echo "Done"
}

trap cleanup EXIT INT TERM

echo "=== Starting logger node ==="
python3 "$SCRIPT_DIR/logger_node.py" --drones "$NUM_DRONES" &
LOGGER_PID=$!

sleep 1

echo "=== Starting main drone via mission_launch ==="
MISSION_DIR="$SCRIPT_DIR/../resources/scripts/Mission"
python3 "$MISSION_DIR/mission_launch.py" "$MISSION_DIR/mission_01.json" &
MAIN_PID=$!

echo "=== Starting helper drones ==="
python3 "$SCRIPT_DIR/helper_drones.py" &
HELPER_PID=$!

echo ""
echo "  Logger PID:       $LOGGER_PID"
echo "  Main drone PID:   $MAIN_PID"
echo "  Helper drones PID: $HELPER_PID"
echo ""

FAILED=0
wait $MAIN_PID || FAILED=1
wait $HELPER_PID || FAILED=1

echo "Flight missions finished, stopping logger..."
kill $LOGGER_PID 2>/dev/null || true
wait $LOGGER_PID 2>/dev/null || true
