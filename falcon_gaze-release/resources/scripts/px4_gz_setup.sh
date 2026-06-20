#!/bin/bash

# === PX4 root (adjust if needed) ===
PX4_DIR=~/PX4-Autopilot

# === PX4 Gazebo paths ===
export GZ_SIM_RESOURCE_PATH=""
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/worlds
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models

# === Gazebo plugin paths (important for PX4 bridges) ===
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PX4_DIR/Tools/simulation/gz/plugins/led_controller/build
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PX4_DIR/build/px4_sitl_default/build_gazebo

# === Single-machine networking (loopback only) ===
# Everything (gz sim, PX4 gz_bridge, ROS image/LED bridges, MAVSDK) runs on this
# one host. Forcing gz-transport and ROS 2 DDS onto the loopback interface avoids
# "Network is unreachable" multicast-discovery failures when the external NIC
# (e.g. Wi-Fi) has no usable multicast route. Requires loopback multicast, which
# run_all.sh enables. Keeping sim and clients on the SAME setting is essential —
# a mismatch makes clients unable to discover the already-running sim.
export GZ_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1

# === Optional but useful ===
export PX4_DIR=$PX4_DIR

echo "PX4 + Gazebo environment loaded"
echo "GZ_SIM_RESOURCE_PATH:"
echo $GZ_SIM_RESOURCE_PATH
echo "GZ_SIM_SYSTEM_PLUGIN_PATH:"
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
