#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$SCRIPT_DIR/resources"
PX4_GZ="$HOME/PX4-Autopilot/Tools/simulation/gz"
PX4_MODELS="$PX4_GZ/models"
PX4_WORLDS="$PX4_GZ/worlds"

if [[ ! -d "$SRC" ]]; then
    echo "ERROR: resources directory not found at: $SRC" >&2
    exit 1
fi

if [[ ! -d "$PX4_GZ" ]]; then
    echo "ERROR: PX4 Gazebo directory not found at: $PX4_GZ" >&2
    exit 1
fi

if [[ ! -d "$PX4_MODELS/x500_base" || ! -d "$PX4_MODELS/x500_mono_cam" || ! -d "$PX4_WORLDS" ]]; then
    echo "ERROR: PX4 Gazebo model/world folders are incomplete." >&2
    echo "Build PX4 SITL once first: cd ~/PX4-Autopilot && make px4_sitl gz_x500" >&2
    exit 1
fi

echo "copying Gazebo world..."
cp -r "$SRC/worlds/media" "$SRC/worlds/baylands_custom.config" "$SRC/worlds/baylands_custom.sdf" "$PX4_WORLDS/"

echo "copying Gazebo x500 model..."
cp "$SRC/x500_base/model.sdf" "$PX4_MODELS/x500_base/"
cp "$SRC/x500_mono_cam/model.config" "$SRC/x500_mono_cam/model.sdf" "$PX4_MODELS/x500_mono_cam/"

echo "copying Gazebo led plugin..."
mkdir -p "$PX4_GZ/plugins"
cp -r "$SRC/plugins/led_controller" "$PX4_GZ/plugins/"

echo "Copying complete."
