# Iteration 7 — `--planner ompl|cumotion` bringup flag (+ grip-in-place experiment)

Date: 2026-07-14
Repos touched: `edi_bottle_picking` (bringup script, this doc); `edi_isaacsim`
(`--grip-in-place` experimental flag).

## bringup_sim_stack.sh changes

New `--planner ompl|cumotion` flag (default `ompl` = byte-identical behavior):

- validated like `--insertion-mode`; expands to `use_cumotion:=true` on the moveit
  launch and `planning_pipeline:=isaac_ros_cumotion` on conveyor_feeding;
- new tmux window **`cumotion`** running `cumotion_planner.launch.py`, with a readiness
  poll on the `cumotion/move_group` action (300 s timeout — tolerates the one-time
  curobo JIT recompile after torch/driver changes);
- moveit-mode `padframe` runs `add_pad -p pad_as_marker:=true` under cumotion (the pad
  must not be a world object for cuMotion); dp-mode prints a warning that
  `diff_physics launch.yaml` does not yet forward `pad_as_marker` (follow-up);
- help text + summary echoes updated; deliberately NO `--insertion-mode cumotion`
  (planner choice is orthogonal to insertion strategy, per plan D2).

## Verification

Single command against a fresh Isaac:
`bringup_sim_stack.sh --planner cumotion --insertion-mode moveit --no-debug
--bottle-picking-iterations 2` — all windows up
(`control moveit velbridge vacbridge padframe cumotion pick autocont`), readiness gate
passed, **both cycles completed the insertion segment** (cycle 1 via the GUARD2
natural-seed retry, cycle 2 directly). Default invocation unchanged (planner=ompl).

## grip-in-place experiment (user-prompted; NOT adopted)

The user observed a ~90° orientation snap when the vacuum "closes". Root cause: the
FixedJoint attach runs in seat-once-then-bond mode (`GRIP_IN_PLACE=False` in
`simplified_ur5_scene.py`) — the bottle is first teleported to the canonical gripper
pose (spin −93.7°, grip_offset 0.012), i.e. **verticalized**, preserving the geometry the
insertion calibration assumes. New CLI flag `--grip-in-place` bonds at the live pose
instead (no snap).

Result of a 2-cycle cumotion run with `--grip-in-place`: cycles complete, but placement
degrades to a consistent **15.9 mm bias with the bottle lowered ~90° to vertical**
(resting on the socket rim, z 0.999–1.002 vs 0.9900). Explanation: bottles lie
horizontal in the box; the top-down grasp holds them sideways; the insertion uses a
FIXED wrist orientation + offset (`moveit_insert_orientation_xyzw`,
`moveit_insert_offset_xyz=[0.015,0,0.09]`) calibrated for the *seated* (verticalized)
bottle. In-place grasping therefore requires **grasp-aware insertion** — deriving the
wrist orientation/offset from the measured bottle-in-hand transform at attach — plus
offset recalibration. Deferred as a follow-up feature; `--grip-in-place` stays
experimental and default-off, and comparisons continue with the validated seat-snap
geometry.

## Verdict

**PASS** for the iteration goal: one-command cuMotion bringup works end-to-end.
Follow-ups: forward `pad_as_marker` through `diff_physics launch.yaml` (dp+cumotion),
grasp-aware insertion for grip-in-place.
