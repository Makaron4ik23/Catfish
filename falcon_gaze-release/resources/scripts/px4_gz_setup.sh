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

# === Optional but useful ===
export PX4_DIR=$PX4_DIR

echo "PX4 + Gazebo environment loaded"
echo "GZ_SIM_RESOURCE_PATH:"
echo $GZ_SIM_RESOURCE_PATH
echo "GZ_SIM_SYSTEM_PLUGIN_PATH:"
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
