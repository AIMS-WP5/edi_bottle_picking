#!/usr/bin/env bash
#
# bringup_sim_stack.sh -- launch the WS_EDI Isaac-sim ROS test stack in one tmux session,
# one window per ROS node. Each window streams its node's output LIVE and also tees it to a
# per-node log file under $LOGDIR (so it's both on screen and inspectable later).
#
# Start the Isaac scene first:  python simplified_ur5_scene.py --omnigraph  (press Play),
# THEN run this script. (It does not touch Isaac -- you start/stop that yourself.)
#
# Usage:
#   bringup_sim_stack.sh [--model NAME] [--steps N] [--collision-check true|false]
#                        [--gripper TYPE] [--no-pick] [--no-attach] [--best-grasp]
#   bringup_sim_stack.sh down            # Ctrl-C every node and kill the tmux session
#
# Examples:
#   bringup_sim_stack.sh --model 2026_06_xx_model_2 --steps 100      # your new checkpoint
#   bringup_sim_stack.sh                                             # known-good defaults
#   bringup_sim_stack.sh --best-grasp                               # use stand-in /best_grasp pub (not Isaac)
#   bringup_sim_stack.sh down
#
# NB: by default /best_grasp is published by the Isaac OmniGraph (per-bottle grasp poses
# from the scene). --best-grasp instead runs the best_grasp_pub.py stand-in; do NOT use
# both at once -- two publishers on /best_grasp race and the pick target becomes nondeterministic.
#
# NB: no `set -u` -- sourcing ROS/ament setup.bash references unset vars and would abort
# the script under `set -u` (e.g. AMENT_TRACE_SETUP_FILES: unbound variable).
set -o pipefail

# Resolve siblings relative to this script (it lives in edi_bottle_picking/scripts/).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
ENV_HELPER="$SCRIPT_DIR/edi_ros_env.sh"
BEST_GRASP="$SCRIPT_DIR/best_grasp_pub.py"
SESSION="edisim"
LOGDIR="/tmp/edi_sim_logs"

# defaults = a known-good baseline; override --model/--steps for a different checkpoint
MODEL_NAME="2026_05_25_model_1"
STEP_COUNT="400"
COLLISION_CHECK="false"
GRIPPER_TYPE="vacuum"
RUN_PICK=1
ATTACH=1
# Optional stand-in vision publisher on /best_grasp. Disabled by default: the Isaac
# OmniGraph now publishes /best_grasp from the scene's bottle coordinates, so running the
# stand-in too would put two racing publishers on the topic. Enable with --best-grasp only
# when driving the pick from the stand-in instead of Isaac.
RUN_BESTGRASP=0

# ---------- subcommand: tear the stack down ----------
if [[ "${1:-}" == "down" || "${1:-}" == "--down" || "${1:-}" == "stop" ]]; then
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        for w in $(tmux list-windows -t "$SESSION" -F '#{window_name}'); do
            tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null || true
        done
        sleep 3
        tmux kill-session -t "$SESSION" 2>/dev/null || true
        echo "stack stopped (tmux session '$SESSION' killed). Isaac left running."
    else
        echo "no tmux session '$SESSION'."
    fi
    exit 0
fi

# ---------- args ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -m|--model)         MODEL_NAME="$2"; shift 2;;
        -s|--steps)         STEP_COUNT="$2"; shift 2;;
        --collision-check)  COLLISION_CHECK="$2"; shift 2;;
        --gripper)          GRIPPER_TYPE="$2"; shift 2;;
        --no-pick)          RUN_PICK=0; shift;;
        --no-attach)        ATTACH=0; shift;;
        --best-grasp)       RUN_BESTGRASP=1; shift;;
        -h|--help)          awk 'NR>1 && /^#/{sub(/^# ?/,""); print; next} NR>1{exit}' "$0"; exit 0;;
        *) echo "unknown arg: $1 (try --help)" >&2; exit 1;;
    esac
done

command -v tmux >/dev/null || { echo "tmux not installed -> sudo apt install tmux" >&2; exit 1; }
[[ -f "$ENV_HELPER" ]]  || { echo "missing env helper: $ENV_HELPER" >&2; exit 1; }
(( RUN_BESTGRASP )) && { [[ -f "$BEST_GRASP" ]] || { echo "missing grasp stand-in: $BEST_GRASP" >&2; exit 1; }; }
mkdir -p "$LOGDIR"
DISP="${DISPLAY:-:1}"

# source env in THIS shell for the readiness polls below
# shellcheck disable=SC1090
source "$ENV_HELPER" >/dev/null 2>&1 || true

if ! timeout 5 ros2 topic echo /clock --once >/dev/null 2>&1; then
    echo "WARNING: /clock not detected -- is the Isaac scene playing? Continuing anyway."
fi

# ---------- helpers ----------
# Open a tmux window named $1 that sources the env, runs $2 with live + tee'd output, and
# leaves an (env-sourced) shell at the prompt afterwards so you can up-arrow to re-run.
_made=0
newwin() {
    local name="$1" cmd="$2" log="$LOGDIR/$1.log"
    local inner="export DISPLAY=$DISP; source $ENV_HELPER; echo '--- $name ---'; stdbuf -oL -eL $cmd 2>&1 | tee $log; echo; echo '=== $name stopped -- up-arrow to re-run ==='"
    if (( _made == 0 )); then
        tmux new-session -d -s "$SESSION" -n "$name"; _made=1
    else
        tmux new-window -t "$SESSION" -n "$name"
    fi
    tmux send-keys -t "$SESSION:$name" "$inner" C-m
}

wait_for() {  # wait_for <desc> <test-cmd> [timeout_s]
    local desc="$1" test="$2" to="${3:-90}" t0=$SECONDS
    echo -n "  waiting for $desc "
    while ! eval "$test" >/dev/null 2>&1; do
        sleep 1; echo -n "."
        if (( SECONDS - t0 > to )); then echo " TIMEOUT (continuing anyway)"; return 1; fi
    done
    echo " ok"
}

tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "== phase 1: control + MoveIt + bridges =="
newwin control   "ros2 launch edi_moveit_config edi_ur_control.launch.py ur_type:=ur5e sim_isaac:=true gripper_type:=$GRIPPER_TYPE use_sim_time:=true initial_joint_controller:=joint_trajectory_controller"
newwin moveit    "ros2 launch edi_moveit_config edi_ur_moveit.launch.py ur_type:=ur5e sim_isaac:=true gripper_type:=$GRIPPER_TYPE use_sim_time:=true launch_rviz:=true"
newwin velbridge "ros2 launch edi_bottle_picking velocity_mode_bridge.launch.py"
newwin vacbridge "ros2 launch edi_bottle_picking vacuum_gripper_bridge.launch.py"

wait_for "controllers" "timeout 6 ros2 control list_controllers 2>/dev/null | grep -q 'joint_trajectory_controller.*active'" 90
wait_for "move_group"  "ros2 node list 2>/dev/null | grep -q /move_group" 90

echo "== phase 2: DP node ($MODEL_NAME, steps=$STEP_COUNT)$( ((RUN_BESTGRASP)) && echo ' + grasp stand-in') =="
newwin dp        "ros2 launch diff_physics launch.yaml model_run:=true model_name:=$MODEL_NAME step_count:=$STEP_COUNT collision_check:=$COLLISION_CHECK use_sim_time:=true"
# /best_grasp is normally published by the Isaac OmniGraph (per-bottle grasp poses); the
# stand-in below would be a SECOND publisher on the same topic, so it's opt-in (--best-grasp).
if (( RUN_BESTGRASP )); then
    newwin bestgrasp "python3 -u $BEST_GRASP --ros-args -p use_sim_time:=true"
fi

if (( RUN_PICK )); then
    wait_for "/object_point (DP add_pad)" "timeout 5 ros2 topic echo /object_point --once 2>/dev/null | grep -q 'x:'" 60
    echo "== phase 3: pick (conveyor_feeding) =="
    newwin pick "ros2 launch edi_bottle_picking conveyor_feeding.launch.py use_sim_time:=true"
fi

echo
echo "tmux session '$SESSION' is up."
echo "  model=$MODEL_NAME  steps=$STEP_COUNT  collision_check=$COLLISION_CHECK  gripper=$GRIPPER_TYPE"
echo "  windows: control moveit velbridge vacbridge dp$( ((RUN_BESTGRASP)) && echo ' bestgrasp')$( ((RUN_PICK)) && echo ' pick')"
echo "  logs:    $LOGDIR/<window>.log"
echo "  navigate: Ctrl-b w (picker) | Ctrl-b <n> | Ctrl-b n / Ctrl-b p"
echo "  detach:   Ctrl-b d        tear down: $0 down"
echo "  the 'pick' window parks at the first RViz 'Next' prompt -- step it with RvizVisualToolsGui."

if [[ -n "${TMUX:-}" ]]; then
    echo "(already inside tmux; not auto-attaching. switch: tmux switch-client -t $SESSION)"
elif (( ATTACH )); then
    sleep 1; tmux attach -t "$SESSION"
else
    echo "attach with: tmux attach -t $SESSION"
fi
