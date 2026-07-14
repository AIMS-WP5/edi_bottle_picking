# cuMotion integration — status: COMPLETE (2026-07-14) & next steps

The full plan (`cumotion-integration-plan.md`, iterations 0–8, docs
`iteration-00` … `iteration-08` in this directory) is implemented, verified against
Isaac Sim 6.0.1, and pushed on `feature/cumotion-integration` in all five repos
(`edi_bottle_picking`, `edi_robot_control`, `edi_ur`, `edi_ai_moveit_servo`,
`edi_isaacsim`).

**Use it:** `bringup_sim_stack.sh --planner cumotion [--insertion-mode dp|moveit]`
(Isaac first, as always). Headline result: placement parity with OMPL in moveit mode
(0.1 mm mean, 5/5), DP-range accuracy in dp mode; full comparison table in
`iteration-08-comparison.md`.

## Next steps (in suggested order)

1. **Bottle asset fixups** (`edi_isaacsim`; spec from user, 2026-07-14): re-adopt the
   robo-codegen `bottle_v3.usd` by KEEPING its mesh-accurate collision diameter —
   adjust the bottle's *nominal* diameter if the two disagree — and adding rolling
   friction/damping (bottles currently roll unrealistically long). Also reconcile the
   asset's internal axis convention (~90° about the long axis vs the old asset) or
   adapt `move_bottle_to_gripper`'s `bottle_spin_deg`. Reverted asset:
   `edi_isaacsim@228fdc1`; context in `iteration-08-comparison.md` §Scene asset note.
2. **Grasp-aware insertion** → enables `--grip-in-place` (no attach snap, physical
   suction end-to-end): derive the insertion wrist orientation/offset from the measured
   bottle-in-hand transform at attach instead of the fixed
   `moveit_insert_orientation_xyzw`/`offset_xyz`. Details:
   `iteration-07-bringup-planner-flag.md` §grip-in-place experiment.
3. **Attached-bottle visibility to cuMotion** (`isaac_ros_cumotion_object_attachment`)
   — currently moot because near-contact segments are OMPL-pinned; needed if cuMotion
   should ever plan while carrying near obstacles.
4. **nvblox/ESDF world** for perception-driven obstacles (cumotion node
   `read_esdf_world`, off by default).
5. **Real-robot validation**: cuMotion's native jerk-limited timing
   (`--retime-plans false`) with a tuned `--time-dilation` (0.2 was 2.3× slower than
   TOTG in sim; likely worth it on hardware for smoothness).

Known environment pins & gotchas: `~/.claude` memory `isaac-ros-cumotion-quirks` and
`iteration-06-planner-selection.md` (frame-handling, warp-lang==1.4.2, hybrid
OMPL/cuMotion split rationale).
