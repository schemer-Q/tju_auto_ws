#!/usr/bin/env bash
set -euo pipefail

# Workspace root (script is in workspace/scripts)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_YAML_DEFAULT="$WS_DIR/map/map.yaml"
MAP_YAML="${1:-$MAP_YAML_DEFAULT}"

if [[ ! -f "$MAP_YAML" ]]; then
  echo "[ERROR] Map yaml not found: $MAP_YAML" >&2
  exit 1
fi

# Source workspace environment (shield COLCON_TRACE unset under set -u)
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$WS_DIR/install/setup.bash"
  set -u
else
  echo "[ERROR] install/setup.bash not found. Please run colcon build first." >&2
  exit 1
fi

MAP_PUB="$WS_DIR/install/dynamic_map/lib/dynamic_map/mock_map_pub.py"
PLANNER_BIN="$WS_DIR/install/path_planner/lib/path_planner/planner_node"

if [[ ! -x "$MAP_PUB" ]]; then
  echo "[ERROR] mock_map_pub.py not found or not executable: $MAP_PUB" >&2
  exit 1
fi

# Fix for NVIDIA GLSL errors in RViz
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export QT_QPA_PLATFORM=xcb
export QT_ENABLE_HIGHDPI_SCALING=0
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/opt/ros/rolling/opt/rviz_ogre_vendor/lib"

# Start processes
"$MAP_PUB" --map_yaml "$MAP_YAML" > /tmp/mock_map.log 2>&1 &
MAP_PID=$!

echo "[INFO] mock_map_pub PID: $MAP_PID (log: /tmp/mock_map.log)"
echo "[INFO] Launching planner and RViz via ros2 launch..."

# Launch planner and RViz using ros2 launch
# This replaces calling planner_node binary directly and calling rviz2 manually
# The launch file handles both, and inherits the exported environment variables
ros2 launch path_planner planner_launch.py &
LAUNCH_PID=$!

grant_exit_cleanup() {
  echo "[INFO] Stopping demo..."
  kill "$MAP_PID" "$LAUNCH_PID" 2>/dev/null || true
}
trap grant_exit_cleanup EXIT INT TERM

# Wait for background processes
wait "$LAUNCH_PID"
