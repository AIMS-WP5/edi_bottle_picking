#!/usr/bin/env bash
#
# bringup_sim_stack.sh -- launch the WS_EDI Isaac-sim ROS test stack in one tmux session,
# one window per ROS node. Each window streams its node's output LIVE and also tees it to a
# per-node log file under $LOGDIR (so it's both on screen and inspectable later).
#
# DEPRECATED WORLD. edi_isaacsim is being retired in favour of robo-codegen-edi (see
# bringup_og_stack.sh). This script keeps working only because it passes pose_set:=isaac to
# conveyor_feeding: the scenario's DEFAULT named poses are now the real EDI cell's
# (above_box_2 / near_box), which sit on the OPPOSITE SIDE of the robot from this scene's box.
# Drop that flag and the arm sweeps away from the box and picks nothing.
#
# Start the Isaac scene first:  python simplified_ur5_scene.py --omnigraph  (press Play),
# THEN run this script. (It does not touch Isaac -- you start/stop that yourself.)
#
# Usage:
#   bringup_sim_stack.sh [--model NAME] [--steps N] [--collision-check true|false]
#                        [--max-velocity RAD_S] [--gripper TYPE] [--no-pick] [--no-attach]
#                        [--best-grasp] [--debug|--no-debug] [--bottle-picking-iterations N]
#                        [--insertion-mode dp|moveit] [--planner ompl|cumotion]
#                        [--grasp-aware true|false] [--pick-depth-flush true|false]
#                        [--radius-aware-insert true|false]
#   bringup_sim_stack.sh down            # Ctrl-C every node and kill the tmux session
#
# Examples:
#   bringup_sim_stack.sh --model 2026_06_xx_model_2 --steps 100      # your new checkpoint
#   bringup_sim_stack.sh                                             # known-good defaults
#   bringup_sim_stack.sh --best-grasp                               # use stand-in /best_grasp pub (not Isaac)
#   bringup_sim_stack.sh --no-debug                                 # run full cycle, no RViz 'Next' clicks
#   bringup_sim_stack.sh --bottle-picking-iterations 5             # run only 5 pick/insert cycles
#   bringup_sim_stack.sh --insertion-mode moveit                   # MoveIt comparison insert (no DP node)
#   bringup_sim_stack.sh down
#
# --planner cumotion: route the task's free-space joint-space planning through NVIDIA cuMotion
# (GPU) instead of OMPL. Adds a 'cumotion' window (cumotion_planner_node via
# edi_moveit_config/cumotion_planner.launch.py), starts move_group with use_cumotion:=true,
# runs add_pad with pad_as_marker:=true (cuMotion mirrors the world without the ACM, so the
# pad must not be a scene object), and passes planning_pipeline:=isaac_ros_cumotion to
# conveyor_feeding. The pick / retreat / insertion segments still pin themselves to OMPL
# (near-contact start states + exact-branch joint goals; see conveyor_feeding_utils
# PipelineScope). There is deliberately NO --insertion-mode cumotion: planner choice is
# orthogonal to insertion strategy. NB: the first cumotion start after a torch/driver change
# re-JITs CUDA kernels (~4 min) -- the readiness wait tolerates it.
#
# --grasp-aware true (moveit insertion mode only): derive the insert EE pose from the measured
# bottle-in-hand transform (grasp_in_hand, published by Isaac at the suction bond) instead of
# the fixed calibrated pose -- so the bottle ends upright over the socket however it sits in
# the gripper. Pair with Isaac's --grip-in-place for physical suction end-to-end (no seat
# snap). With the default seat-snap the derived pose reproduces the fixed one (delta logged
# per insertion). Default false = byte-identical legacy behaviour.
#
# --pick-depth-flush true: press the pick grasp target one suction-tip-compliance deeper so the
# rigid cup tip meets the bottle surface flush. Intended together with Isaac's
# --best-grasp-at-surface (best_grasp published at the bottle surface instead of its centroid).
# Default false = pick target unchanged.
#
# --radius-aware-insert true (moveit insertion mode only; ignored when --grasp-aware true):
# derive the fixed-mode insert pose through the canonical radius-aware transform so it tracks
# the configured bottle_radius. Reproduces the fixed calibrated pose at the baseline bottle
# (delta logged per insertion). Default false = the literal fixed moveit_insert_offset_xyz pose.
#
# --insertion-mode moveit: run the MoveIt comparison test instead of the DP velocity segment.
# The DP node is NOT launched (the insertion is MoveIt position-controlled + a Cartesian
# descent), and conveyor_feeding runs with insertion_mode:=moveit. For a clean comparison,
# start Isaac with the DP socket bias disabled:
#   python simplified_ur5_scene.py --omnigraph --pad-adj-x 0 --pad-adj-y 0
# (this script does NOT launch Isaac -- you start/stop that yourself; see above).
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
# Collision filter's max joint velocity (rad/s); restates the node default 3.0. Set low
# (e.g. 0.01) to force the velocity clamp to engage -- useful for demoing that over-limit
# commands are clamped (not aborted). Only meaningful with --collision-check true.
MAX_VELOCITY="3.0"
GRIPPER_TYPE="vacuum"
# Debug stepping: true = pause for a 'Next' click in RViz between each pick/insert stage;
# false = run the full scenario and keep cycling unattended. Toggle live while running with:
#   ros2 topic pub --once /conveyor_feeding/debug std_msgs/msg/Bool "{data: false}"   (or true)
DEBUG="true"
# Number of pick/insert cycles conveyor_feeding runs. -1 (default) keeps the value from
# conveyor_feeding_config.yaml ('iterations', currently 50); any value >= 0 overrides it.
BOTTLE_PICKING_ITERATIONS="-1"
# Insertion strategy passed through to conveyor_feeding. "dp" (default) = NN velocity segment;
# "moveit" = comparison test (MoveIt above-socket move + Cartesian descent). In "moveit" the DP
# node is skipped and a reminder to start Isaac with --pad-adj-x 0 --pad-adj-y 0 is printed.
INSERTION_MODE="dp"
# Planning pipeline for the task's free-space moves: "ompl" (default, unchanged behavior) or
# "cumotion" (NVIDIA isaac_ros_cumotion via the move_group pipeline; see --planner note above).
PLANNER="ompl"
# A/B knobs for the cuMotion comparison: --retime-plans false executes the planner's own
# time parameterization (cuMotion: jerk-limited) instead of TOTG; --time-dilation scales
# cuMotion's native timing (only meaningful with --retime-plans false; ~0.2 matches the
# sim-gated TOTG scaling).
RETIME_PLANS="true"
TIME_DILATION="0.5"
# Grasp-aware insertion (moveit insertion mode only): derive the insert EE pose from the
# measured bottle-in-hand transform instead of the fixed calibrated pose. See --grasp-aware.
GRASP_AWARE="false"
# Pick-depth flush + radius-aware insert (see the header notes). Default off = legacy behaviour.
PICK_DEPTH_FLUSH="false"
RADIUS_AWARE_INSERT="false"
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
        --max-velocity)     MAX_VELOCITY="$2"; shift 2;;
        --gripper)          GRIPPER_TYPE="$2"; shift 2;;
        --no-pick)          RUN_PICK=0; shift;;
        --no-attach)        ATTACH=0; shift;;
        --best-grasp)       RUN_BESTGRASP=1; shift;;
        --debug)            DEBUG="true"; shift;;
        --no-debug)         DEBUG="false"; shift;;
        --bottle-picking-iterations) BOTTLE_PICKING_ITERATIONS="$2"; shift 2;;
        --insertion-mode)   INSERTION_MODE="$2"; shift 2;;
        --planner)          PLANNER="$2"; shift 2;;
        --retime-plans)     RETIME_PLANS="$2"; shift 2;;
        --time-dilation)    TIME_DILATION="$2"; shift 2;;
        --grasp-aware)      GRASP_AWARE="$2"; shift 2;;
        --pick-depth-flush) PICK_DEPTH_FLUSH="$2"; shift 2;;
        --radius-aware-insert) RADIUS_AWARE_INSERT="$2"; shift 2;;
        -h|--help)          awk 'NR>1 && /^#/{sub(/^# ?/,""); print; next} NR>1{exit}' "$0"; exit 0;;
        *) echo "unknown arg: $1 (try --help)" >&2; exit 1;;
    esac
done

case "$INSERTION_MODE" in
    dp|moveit) ;;
    *) echo "invalid --insertion-mode '$INSERTION_MODE' (expected: dp | moveit)" >&2; exit 1;;
esac
case "$PLANNER" in
    ompl) USE_CUMOTION="false"; PLANNING_PIPELINE="ompl";;
    cumotion) USE_CUMOTION="true"; PLANNING_PIPELINE="isaac_ros_cumotion";;
    *) echo "invalid --planner '$PLANNER' (expected: ompl | cumotion)" >&2; exit 1;;
esac
case "$GRASP_AWARE" in
    true|false) ;;
    *) echo "invalid --grasp-aware '$GRASP_AWARE' (expected: true | false)" >&2; exit 1;;
esac
if [[ "$GRASP_AWARE" == "true" && "$INSERTION_MODE" != "moveit" ]]; then
    echo "WARNING: --grasp-aware true only affects --insertion-mode moveit (current: $INSERTION_MODE)" >&2
fi
case "$PICK_DEPTH_FLUSH" in
    true|false) ;;
    *) echo "invalid --pick-depth-flush '$PICK_DEPTH_FLUSH' (expected: true | false)" >&2; exit 1;;
esac
case "$RADIUS_AWARE_INSERT" in
    true|false) ;;
    *) echo "invalid --radius-aware-insert '$RADIUS_AWARE_INSERT' (expected: true | false)" >&2; exit 1;;
esac
if [[ "$RADIUS_AWARE_INSERT" == "true" && "$INSERTION_MODE" != "moveit" ]]; then
    echo "WARNING: --radius-aware-insert true only affects --insertion-mode moveit (current: $INSERTION_MODE)" >&2
fi
if [[ "$RADIUS_AWARE_INSERT" == "true" && "$GRASP_AWARE" == "true" ]]; then
    echo "WARNING: --radius-aware-insert is ignored when --grasp-aware true (grasp-aware takes precedence)" >&2
fi

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

# ---------- decluttered RViz config ----------
# Generate a decluttered copy of edi_robot_description's base.rviz at launch time and hand
# it to the moveit launch via rviz_config:= -- turns off TF axes + name labels and the
# MotionPlanning query-goal-state (orange goal ghost) render. The shared default base.rviz is
# left untouched; regenerated every run so it always tracks the installed base.rviz.
RVIZ_CFG=""
BASE_RVIZ="$(ros2 pkg prefix --share edi_robot_description 2>/dev/null)/rviz/base.rviz"
if [[ -f "$BASE_RVIZ" ]]; then
    RVIZ_CFG="$LOGDIR/base_decluttered.rviz"
    sed -e 's/Show Axes: true/Show Axes: false/' \
        -e 's/Show Names: true/Show Names: false/' \
        -e 's/Query Goal State: true/Query Goal State: false/' \
        "$BASE_RVIZ" > "$RVIZ_CFG"
    echo "  RViz: decluttered config -> $RVIZ_CFG (TF axes/labels off, query-goal-state off)"
else
    echo "  WARNING: base.rviz not found ($BASE_RVIZ) -- RViz uses its default config" >&2
fi

echo "== phase 1: control + MoveIt + bridges =="
newwin control   "ros2 launch edi_moveit_config edi_ur_control.launch.py ur_type:=ur5e sim_isaac:=true gripper_type:=$GRIPPER_TYPE use_sim_time:=true initial_joint_controller:=joint_trajectory_controller"
newwin moveit    "ros2 launch edi_moveit_config edi_ur_moveit.launch.py ur_type:=ur5e sim_isaac:=true gripper_type:=$GRIPPER_TYPE use_sim_time:=true launch_rviz:=true use_cumotion:=$USE_CUMOTION${RVIZ_CFG:+ rviz_config:=$RVIZ_CFG}"
newwin velbridge "ros2 launch edi_bottle_picking velocity_mode_bridge.launch.py"
newwin vacbridge "ros2 launch edi_bottle_picking vacuum_gripper_bridge.launch.py"

wait_for "controllers" "timeout 6 ros2 control list_controllers 2>/dev/null | grep -q 'joint_trajectory_controller.*active'" 90
wait_for "move_group"  "ros2 node list 2>/dev/null | grep -q /move_group" 90

# The DP node drives the velocity insertion AND (via the diff_physics launch) runs add_pad,
# which populates the planning scene with the base_table workspace collision (constrains the
# planner) and the madara_pad (collision-EXCLUDED, RViz-visible). In MoveIt mode the DP
# controller is skipped, but we still run add_pad standalone so the base_table is present --
# without it MoveIt routes the arm through the empty workspace and plans long twist-around paths.
if [[ "$INSERTION_MODE" == "moveit" ]]; then
    echo "== phase 2: DP node SKIPPED (insertion_mode=moveit); running add_pad for workspace collision =="
    # With cuMotion, the pad must be an RViz marker, not a scene object (cuMotion mirrors the
    # world without the ACM, so a pad object at the socket target blocks every nearby plan).
    if [[ "$PLANNER" == "cumotion" ]]; then
        newwin padframe  "ros2 run diff_physics add_pad --ros-args -p use_sim_time:=true -p pad_as_marker:=true"
    else
        newwin padframe  "ros2 run diff_physics add_pad --ros-args -p use_sim_time:=true"
    fi
else
    echo "== phase 2: DP node ($MODEL_NAME, steps=$STEP_COUNT)$( ((RUN_BESTGRASP)) && echo ' + grasp stand-in') =="
    PAD_AS_MARKER=$( [[ "$PLANNER" == "cumotion" ]] && echo "true" || echo "false" )
    newwin dp        "ros2 launch diff_physics launch.yaml model_run:=true model_name:=$MODEL_NAME step_count:=$STEP_COUNT collision_check:=$COLLISION_CHECK max_velocity:=$MAX_VELOCITY use_sim_time:=true pad_as_marker:=$PAD_AS_MARKER"
fi

# cuMotion planner node (exactly ONE instance). Started before the pick so the readiness
# poll below can gate conveyor_feeding on the action server.
if [[ "$PLANNER" == "cumotion" ]]; then
    echo "== phase 2b: cuMotion planner node =="
    newwin cumotion "ros2 launch edi_moveit_config cumotion_planner.launch.py use_sim_time:=true time_dilation_factor:=$TIME_DILATION"
    # First start after a torch/driver change re-JITs curobo's CUDA kernels (~4 min);
    # normally ready in ~30 s (log line: 'cuMotion is ready for planning queries!').
    wait_for "cumotion/move_group action" "ros2 action list 2>/dev/null | grep -q cumotion/move_group" 300
fi
# /best_grasp is normally published by the Isaac OmniGraph (per-bottle grasp poses); the
# stand-in below would be a SECOND publisher on the same topic, so it's opt-in (--best-grasp).
if (( RUN_BESTGRASP )); then
    newwin bestgrasp "python3 -u $BEST_GRASP --ros-args -p use_sim_time:=true"
fi

if (( RUN_PICK )); then
    # No goal-topic gate here: conveyor_feeding runs its MoveIt pick and moves to ai_start2
    # before the DP segment, so /socket_center (published by the Isaac OmniGraph) is live well
    # before run_dp_segment() reads it. (The old /object_point wait was a stale check -- that
    # topic was renamed to /socket_center -- so it always burned its full 60 s timeout.)
    echo "== phase 3: pick (conveyor_feeding, insertion_mode=$INSERTION_MODE) =="
    newwin pick "ros2 launch edi_bottle_picking conveyor_feeding.launch.py pose_set:=isaac use_sim_time:=true debug:=$DEBUG iterations:=$BOTTLE_PICKING_ITERATIONS insertion_mode:=$INSERTION_MODE planning_pipeline:=$PLANNING_PIPELINE retime_plans:=$RETIME_PLANS grasp_aware_insertion:=$GRASP_AWARE pick_depth_flush:=$PICK_DEPTH_FLUSH moveit_insert_radius_aware:=$RADIUS_AWARE_INSERT"
    # (No 'autocont' window any more. manipulator_interface::cartesian_goal() used to issue an
    # UNCONDITIONAL world_marker_->prompt() before executing the cartesian plan, which no-debug
    # runs could only escape by publishing buttons[2] ('Continue') on /rviz_visual_tools_gui at
    # 1 Hz to latch rviz_visual_tools into autonomous mode. Those prompts now go through the
    # shared manipulator_interface::DebugStepGate and honour debug:= like every other stage.)
fi

echo
if [[ "$BOTTLE_PICKING_ITERATIONS" == "-1" ]]; then ITERS_DISP="config default"; else ITERS_DISP="$BOTTLE_PICKING_ITERATIONS"; fi
echo "tmux session '$SESSION' is up."
echo "  insertion_mode=$INSERTION_MODE  planner=$PLANNER  grasp_aware=$GRASP_AWARE  pick_depth_flush=$PICK_DEPTH_FLUSH  radius_aware_insert=$RADIUS_AWARE_INSERT  model=$MODEL_NAME  steps=$STEP_COUNT  collision_check=$COLLISION_CHECK  max_velocity=$MAX_VELOCITY  gripper=$GRIPPER_TYPE  debug=$DEBUG  bottle_picking_iterations=$ITERS_DISP"
echo "  windows: control moveit velbridge vacbridge$( [[ $INSERTION_MODE == moveit ]] && echo ' padframe' || echo ' dp')$( [[ $PLANNER == cumotion ]] && echo ' cumotion')$( ((RUN_BESTGRASP)) && echo ' bestgrasp')$( ((RUN_PICK)) && echo ' pick')"
if [[ "$INSERTION_MODE" == "moveit" ]]; then
    echo "  NOTE: MoveIt comparison mode -- DP node not launched. For a clean comparison start Isaac with:"
    echo "        python simplified_ur5_scene.py --omnigraph --pad-adj-x 0 --pad-adj-y 0"
fi
echo "  logs:    $LOGDIR/<window>.log"
echo "  navigate: Ctrl-b w (picker) | Ctrl-b <n> | Ctrl-b n / Ctrl-b p"
echo "  detach:   Ctrl-b d        tear down: $0 down"
if [[ "$DEBUG" == "true" ]]; then
    echo "  the 'pick' window parks at the first RViz 'Next' prompt -- step it with RvizVisualToolsGui,"
    echo "  or free-run mid-wait: ros2 topic pub --once /conveyor_feeding/debug std_msgs/msg/Bool '{data: false}'"
fi

if [[ -n "${TMUX:-}" ]]; then
    echo "(already inside tmux; not auto-attaching. switch: tmux switch-client -t $SESSION)"
elif (( ATTACH )); then
    sleep 1; tmux attach -t "$SESSION"
else
    echo "attach with: tmux attach -t $SESSION"
fi
