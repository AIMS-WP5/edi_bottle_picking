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

source /opt/ros/humble/setup.bash
[ -f "$EDI_WS_ROOT/install/setup.bash" ] && source "$EDI_WS_ROOT/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=66
# UDP-only Fast-DDS profile (NVIDIA/Isaac). Keep an already-set value; otherwise use the
# conventional location if it exists.
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && [ -f "$HOME/workspaces/humble_ws/fastdds.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/workspaces/humble_ws/fastdds.xml"
fi
