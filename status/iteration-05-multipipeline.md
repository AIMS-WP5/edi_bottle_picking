# Iteration 5 — move_group multi-pipeline conversion (OMPL default + opt-in cuMotion)

Date: 2026-07-14
Repos touched: `edi_ur` (edi_moveit_config launch), `edi_bottle_picking` (this doc).

## Change

`edi_ur_moveit.launch.py`: the flat single-pipeline OMPL params (legacy `move_group`
namespace) became the multi-pipeline layout —
`planning_pipelines: ["ompl"]` + `default_planning_pipeline: "ompl"` + per-pipeline
config nested under each name. New launch arg **`use_cumotion` (default false)** appends
an `isaac_ros_cumotion` pipeline loading NVIDIA's shipped
`isaac_ros_cumotion_moveit/config/isaac_ros_cumotion_planning.yaml`
(plugin `isaac_ros_cumotion_moveit/CumotionPlanner`, num_steps 32). With the default
`false`, move_group never touches the isaac_ros packages — stack works without them.
The RViz node's params were updated to the same dict (it referenced the old variable).

## Verification (live stack: Isaac 6.0.1 + `--insertion-mode moveit --no-pick`)

- **Default (`use_cumotion` unset)**: move_group loads pipeline `ompl` only; plan-only
  MoveGroup goal via `/move_action` with `pipeline_id=ompl` → SUCCESS (5 pts, 0.021 s).
- **`use_cumotion:=true`** (+ one cumotion_planner_node): startup logs show both
  pipelines — `Using planning interface 'OMPL'` and `'Generate minimum-jerk trajectories
  using NVIDIA Isaac ROS cuMotion'`. Same goal with
  `pipeline_id=isaac_ros_cumotion` → **SUCCESS, 32 points, planning_time 0.120 s**
  (cuMotion signature); `pipeline_id=ompl` still SUCCESS (default unchanged).

Note for client code (Iteration 6): the MoveGroup action of move_group is `/move_action`
(MoveIt 2 naming); `cumotion/move_group` is the *standalone* planner node's server.

## Verdict

**PASS.** cuMotion is now a selectable move_group planning pipeline; existing behavior
is bit-identical when not requested. Next: `planning_pipeline` param in
manipulator_interface + TOTG gating.
