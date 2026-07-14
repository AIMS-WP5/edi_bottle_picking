# Iteration 8 — Validation matrix: cuMotion vs OMPL across both insertion modes

Date: 2026-07-14
Repos touched: `edi_bottle_picking` (ai_start2 handoff pin, retime/time-dilation knobs,
this doc), `edi_ai_moveit_servo` (launch.yaml pad_as_marker forwarding), `edi_isaacsim`
(bottle asset revert — see below).

All runs: Isaac 6.0.1 (env_isaacsim6), fresh scene per cell, one-command
`bringup_sim_stack.sh` (Iteration 7 flag), 5 cycles per cell (A/B cell: 3), placement
from Isaac dp-target records, per-cell logs archived under `/tmp/edi_sim_logs/matrix_*`.
Scene assets: fixed-collision pad + ORIGINAL bottle (see asset note).

## Results

| cell | placed | XY error (mm, per cycle) | mean | cycle sim time | mean plan latency |
|------|--------|--------------------------|------|----------------|-------------------|
| ompl × moveit | 5/5 | 0.1, 0.0, 0.1, 0.0, 0.0 | **0.1** | 31.2 s | 0.024 s (42 plans) |
| cumotion × moveit | 5/5 | 0.1, 0.0, 0.2, 0.2, 0.0 | **0.1** | 36.6 s | 0.117 s (42 plans) |
| ompl × dp | 5/5 | 10.7, 2.3, 7.5, 4.1, 2.6 | **5.4** | 36.0 s | 0.022 s (35 plans) |
| cumotion × dp | 5/5 | 10.6, 6.2, 15.9, 4.0, 5.3 | **8.4** | 40.4 s | 0.132 s (35 plans) |
| cumotion × moveit, retime_plans=false (dilation 0.2) | 3/3 | 0.0, 0.0, 0.1 | **0.0** | 83.1 s | — |

Headlines:
- **cuMotion reaches full placement parity with OMPL in moveit mode** (0.1 mm mean both,
  5/5 both). dp mode: 8.4 vs 5.4 mm — within the DP policy's cycle-to-cycle variance
  (per-cycle ranges overlap; the NN segment dominates dp accuracy, not the planner).
- **Plan latency**: OMPL ~22–24 ms vs cuMotion ~117–142 ms per request (action round-trip
  + GPU trajopt) in this small scene — cuMotion is not a latency win here; its value is
  trajectory quality: constant 32-point minimum-jerk plans with zero retries, vs OMPL's
  waypoint-complexity retry loop firing 25× per 5-cycle run.
- **Cycle time**: cuMotion cells ~12–17% slower (latency + slightly longer smooth paths
  under identical TOTG re-timing).
- **A/B re-timing**: native cuMotion timing works correctly end-to-end (placement
  perfect) but at time_dilation 0.2 is ~2.3× slower than TOTG-at-0.2 — cuMotion's
  jerk-limited profile is far more conservative than time-optimal re-timing.
  **`retime_plans=true` (TOTG) stays the default**; native timing (with a higher
  dilation) is the candidate for real-robot runs where jerk limits matter.

## Fix found by the matrix: DP handoff pin

First cumotion × dp run "placed" 5/5 but at **131.7 mm mean** — the pre-DP move to
`ai_start2` is a named joint target, and the cuMotion planner node re-solves joint
targets as pose IK: the arm reached the right EE pose in a different configuration, and
the start-config-sensitive DP policy ran off-distribution. Fix: the `ai_start2` handoff
is now pinned to OMPL (`PipelineScope`, conveyor_feeding_utils) — also stabilizes the
moveit-mode insertion seed. Rerun: 8.4 mm mean (above).

Also added for the matrix/A-B: `retime_plans` launch arg on conveyor_feeding;
`--retime-plans` + `--time-dilation` bringup flags; `pad_as_marker` forwarded through
`diff_physics launch.yaml` so `--planner cumotion --insertion-mode dp` works.

## Scene asset note (user observations during the matrix)

- The robo-codegen `bottle_v3.usd` was **reverted** (edi_isaacsim `228fdc1`) mid-iteration
  (matrix restarted from scratch afterwards): its plain-cylinder collision (accurate,
  smaller diameter) makes bottles roll unrealistically long in the box (moving pick
  targets), and its internal geometry axes differ ~90° about the bottle's long axis from
  the old asset (invisible at spawn — the bottle is axis-symmetric — but the canonical
  grasp seat then produces a visible 90° twirl at attach). The fixed-collision
  **madara_pad stays adopted**.
- **Follow-up spec (user)**: re-adopt the new bottle by KEEPING the mesh-accurate
  collision diameter (adjust the bottle's nominal diameter if mismatched) and adding
  rolling friction/damping; also reconcile the internal axis convention (or adapt
  `bottle_spin_deg` in move_bottle_to_gripper).

## Open follow-ups (beyond this branch's goal)

1. Bottle asset fixups per the spec above (edi_isaacsim).
2. Grasp-aware insertion (grip-in-place adoption; Iteration 7 doc).
3. Attached-bottle visibility to cuMotion (isaac_ros_cumotion_object_attachment) —
   currently moot: all near-contact segments are OMPL-pinned.
4. nvblox/ESDF world (perception-driven obstacles) — untouched, off by default.
5. Real-robot validation: native cuMotion timing with tuned time_dilation.

## Verdict

**PASS — integration goal met.** cuMotion runs as a first-class planning pipeline for
the bottle-picking task (one-command bringup, both insertion modes, placement parity in
moveit mode, DP-range accuracy in dp mode), with the precision/near-contact segments
deliberately kept on OMPL per the hybrid design. The plan's full iteration ladder
(0 → 8) is complete.
