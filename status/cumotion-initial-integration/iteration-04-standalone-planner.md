# Iteration 4 — Standalone cumotion_planner_node + live smoke test

Date: 2026-07-14
Repos touched: `edi_ur` (edi_moveit_config launch), `edi_bottle_picking` (smoke script,
CMakeLists, this doc). Branch `feature/cumotion-integration`.

## New files

- **`edi_moveit_config/launch/cumotion_planner.launch.py`**: runs
  `generate_cumotion_urdf.py` at launch time (OpaqueFunction; URDF can't drift from the
  live xacro), then starts `cumotion_planner_node` with `robot:=<edi_ur5e.xrdf>` and
  `urdf_path:=<generated>`. Args: `ur_type`, `gripper_type`, `use_sim_time`,
  `time_dilation_factor` (default 0.5; only matters when TOTG re-timing is bypassed).
- **`edi_bottle_picking/scripts/cumotion_smoke_test.py`** (installed): rclpy action
  client on `cumotion/move_group` (moveit_msgs/action/MoveGroup, plan_only). Sends a
  joint goal (current q + delta), a pose goal (`virtual_ee_link` +5 cm z, from tf2), and
  a return joint goal.

## Blocker found & fixed: warp-lang version

`cumotion_planner_node` crashed at startup with `AttributeError: module 'warp' has no
attribute 'torch'` — warp-lang **1.15** (installed in Iteration 2) lazy-loads submodules
and no longer auto-exposes `warp.torch`, which curobo 3.0 relies on (only the planner
node's WorldVoxelCollision path hits it — the Iteration 2 MotionGen probe didn't).
curobo's egg-info declares **no** warp pin, so pip had grabbed latest.
**Fix: `pip install --user warp-lang==1.4.2`** (the curobo-3.0-era version). Verified:
warp 1.4.2 initializes on the RTX 5090 (reports sm_120, CUDA driver 13.2) and
`wp.torch.device_from_torch` works. Supersedes the warp 1.15.0 note in iteration-02.

## Verification (live: Isaac 6.0.1 playing, control stack up via
`bringup_sim_stack.sh --insertion-mode moveit --no-pick --no-debug`, one cumotion node)

```
[joint-goal #1]        error_code=1 points=32 traj_duration=1.56s planning_time=0.116s PASS
[pose-goal(+5cm z) #1] error_code=1 points=32 traj_duration=1.55s planning_time=0.175s PASS
[joint-goal-return #1] error_code=1 points=32 traj_duration=1.56s planning_time=0.129s PASS
```

- Node log: "cuMotion is ready for planning queries!"; startup (post-JIT-cache) ~30 s.
- The pose goal passing confirms end-to-end `virtual_ee_link` tool-frame handling
  (XRDF `tool_frames` + URDF frame + MoveGroup constraint link_name).

## Verdict

**PASS.** cuMotion plans joint and pose goals against the live sim's /joint_states.
Next (Iteration 5): expose it as a move_group planning pipeline.
