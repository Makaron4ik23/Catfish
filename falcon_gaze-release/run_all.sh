#!/bin/bash
source /opt/ros/humble/setup.bash

echo "Starting Follower Mission (Drones 1, 2, 3)..."
python3 examples/follower_mission.py &
FOLLOWER_PID=$!

# Give followers a few seconds to connect to MAVSDK
sleep 5

echo "Starting Leader Mission (Drone 0)..."
python3 resources/scripts/mission_launch.py &
LEADER_PID=$!

# Wait for both scripts to finish
wait $FOLLOWER_PID
wait $LEADER_PID

echo "Both missions finished."
