# Sourced prelude for the WS_EDI Isaac-sim ROS test stack.
#
# Sources ROS + the colcon workspace and pins the DDS config -- rmw_fastrtps_cpp + the
# UDP-only fastdds.xml profile + ROS_DOMAIN_ID 66 -- that every node (and Isaac's ROS 2
# bridge) must share. Non-interactive shells don't inherit these from ~/.bashrc, so we set
# them explicitly; an interactive shell that already set them just gets the same values.
#
# Source it (do not execute):  source .../edi_bottle_picking/scripts/edi_ros_env.sh

# Workspace root, resolved relative to this file (src/edi_bottle_picking/scripts -> WS root).
# Override with EDI_WS_ROOT if the layout differs.
_edi_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
EDI_WS_ROOT="${EDI_WS_ROOT:-$(cd "$_edi_here/../../.." >/dev/null 2>&1 && pwd)}"

# ROS 2 Humble nodes must run on the system python (3.10). An active conda/mamba env
# (e.g. env_isaacsim6, kept active for Isaac Sim work) puts a python 3.12 first on PATH,
# which every `#!/usr/bin/env python3` node then picks up and dies on
# `No module named rclpy._rclpy_pybind11` (rclpy's C extension is cpython-310 only).
# Strip conda from THIS stack's environment; Isaac itself runs in its own shell and is
# unaffected. Must happen before the ROS/workspace setup.bash exports PYTHONPATH.
if [ -n "${CONDA_PREFIX:-}" ] || case ":$PATH:" in *miniforge3*) true;; *) false;; esac; then
    PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v -e miniforge3 -e '/condabin' | paste -sd:)"
    export PATH
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE PYTHONPATH
fi

source /opt/ros/humble/setup.bash
[ -f "$EDI_WS_ROOT/install/setup.bash" ] && source "$EDI_WS_ROOT/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=66
# UDP-only Fast-DDS profile (NVIDIA/Isaac). Keep an already-set value; otherwise use the
# conventional location if it exists.
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && [ -f "$HOME/workspaces/humble_ws/fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/workspaces/humble_ws/fastdds.xml"
fi
