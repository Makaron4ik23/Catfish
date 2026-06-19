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

# === WSL GPU rendering ===
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
export PULSE_SERVER=/mnt/wslg/PulseServer
# WSLg OpenGL goes through Mesa D3D12. Prefer NVIDIA on hybrid GPU systems.
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
export LIBGL_ALWAYS_SOFTWARE=1

# Vulkan currently falls back to llvmpipe in this WSL setup; use OGRE2/OpenGL.
export GZ_RENDER_ENGINE=ogre

# === Optional but useful ===
export PX4_DIR=$PX4_DIR

echo "PX4 + Gazebo environment loaded"
echo "MESA_D3D12_DEFAULT_ADAPTER_NAME:"
echo $MESA_D3D12_DEFAULT_ADAPTER_NAME
echo "GZ_RENDER_ENGINE:"
echo $GZ_RENDER_ENGINE
echo "GZ_SIM_RESOURCE_PATH:"
echo $GZ_SIM_RESOURCE_PATH
echo "GZ_SIM_SYSTEM_PLUGIN_PATH:"
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
