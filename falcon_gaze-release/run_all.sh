#!/bin/bash
source /opt/ros/humble/setup.bash

# Single-machine networking: keep gz-transport + ROS 2 DDS on loopback so
# discovery does not depend on the external NIC's multicast route. Must match
# the sim's settings (resources/scripts/px4_gz_setup.sh sets the same vars).
export GZ_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1

# Ensure the loopback interface can carry multicast discovery traffic. Harmless
# if already enabled; needs sudo once per boot. Skipped automatically if sudo
# is unavailable (e.g. already configured by an admin).
if ! ip route show | grep -q "224.0.0.0/4 dev lo"; then
    echo "Enabling loopback multicast (needs sudo)..."
    sudo ip link set lo multicast on 2>/dev/null || true
    sudo ip route replace 224.0.0.0/4 dev lo 2>/dev/null || true
fi

echo "Cleaning up any old mission scripts..."
pkill -9 -f follower_mission.py || true
pkill -9 -f mission_launch.py || true
pkill -9 -f mavsdk_server || true
sleep 1

echo "Starting Follower Mission (Drones 1, 2, 3)..."
python3 examples/follower_mission.py &
FOLLOWER_PID=$!

# Give followers a few seconds to connect to MAVSDK
sleep 5

echo "Starting Leader Mission (Drone 0) - test scenario from Mission/mission_01.json..."
python3 ../Mission/mission_launch.py ../Mission/mission_01.json --speed 3 &
LEADER_PID=$!

trap 'echo "Stopping missions..."; kill $FOLLOWER_PID $LEADER_PID 2>/dev/null; exit' SIGINT SIGTERM

# Wait for both scripts to finish
wait $FOLLOWER_PID
wait $LEADER_PID

echo "Both missions finished."
