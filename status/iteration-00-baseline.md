# Iteration 0 — Baseline re-validation on Isaac Sim 6.0.1 (env_isaacsim6)

Date: 2026-07-14
Repos touched: `edi_bottle_picking` (branch `feature/cumotion-integration`)
Plan: see `cumotion-integration-plan.md` in this directory.

## Goal

First run of the existing sim stack against the new `env_isaacsim6` mamba env
(Isaac Sim 6.0.1 GA) — previously untested. Validate both insertion modes (dp, moveit)
at 5 cycles each and capture baseline placement metrics before any cuMotion work.

## Regression found & fixed: conda python 3.12 kills the ROS python nodes

**Symptom:** bottles were not being picked — the arm went through the motions but the
vacuum never engaged. Not a grasp-physics regression.

**Root cause:** with `env_isaacsim6` active in the launching shell (now the norm for
Isaac 6 work), its python 3.12 is first on PATH. Every `#!/usr/bin/env python3` ROS node
(`vacuum_gripper_bridge.py`, `velocity_mode_bridge.py`) resolved to py3.12 and died at
import with `No module named 'rclpy._rclpy_pybind11'` (Humble's rclpy C extension is
cpython-310 only). With the vacuum bridge dead, `/vacuum_gripper/command` was
unavailable; `conveyor_feeding` logged *"Isaac vacuum service not available; skipping
sim GRIP command"* and **assumed grasp success**, so the failure was silent apart from
the physical non-pick.

**Fix:** `scripts/edi_ros_env.sh` (the shared prelude every tmux stack window sources)
now strips conda/miniforge from PATH and unsets `CONDA_*`/`PYTHONPATH` before sourcing
ROS. Verified: `python3` → `/usr/bin/python3`, rclpy imports, both bridges start and
serve their topics/services regardless of the invoking shell's conda state.

**Follow-up candidate (not done):** `conveyor_feeding` treats a missing vacuum service
as success in sim; a hard failure (or at least a per-cycle re-warning) would have made
this instantly visible.

## dp mode — 5/5 placed ✔

Isaac: `python simplified_ur5_scene.py --omnigraph --write-after-cycles 5 --auto-exit`
(sim_config.yaml PAD_ADJ x=0.015 y=0.0038 active).
Stack: `bringup_sim_stack.sh --model saved_model_9-06-2026-rand-range --no-debug
--bottle-picking-iterations 5`.
Records: `edi_isaacsim/dptarget-data-20260714-123106.json`; logs archived at
`/tmp/edi_sim_logs/iter0_dp_archive/`.

| cycle | pad | target (x,y) | bottle (x,y) | XY error |
|-------|-----|--------------|--------------|----------|
| 1 | pad_1 | -0.700, 0.048 | -0.7071, 0.0547 | 9.8 mm |
| 2 | pad_2 | -0.700, 0.168 | -0.6980, 0.1684 | 2.0 mm |
| 3 | pad_3 | -0.700, 0.288 | -0.6921, 0.2875 | 7.9 mm |
| 4 | pad_1 | -0.7324, 0.0479 | -0.7306, 0.0441 | 4.2 mm |
| 5 | pad_2 | -0.6523, 0.1878 | -0.6514, 0.1937 | 6.0 mm |

Mean XY error ≈ 6.0 mm — consistent with the known DP per-pad bias characteristics from
the pre-upgrade runs (see `edi_isaacsim/placement_accuracy_*` docs). Clean auto-exit +
record flush worked (validates the 2026-06-20 hang fix on Isaac 6.0.1).

## moveit mode — 5/5 placed ✔

Isaac restarted with `--pad-adj-x 0 --pad-adj-y 0` (bias confirmed 0/0 in log).
Stack: `bringup_sim_stack.sh --insertion-mode moveit --no-debug
--bottle-picking-iterations 5`.
Records: `edi_isaacsim/dptarget-data-20260714-123820.json`; logs archived at
`/tmp/edi_sim_logs/iter0_moveit_archive/`.

| cycle | pad | target (x,y) | bottle (x,y) | XY error |
|-------|-----|--------------|--------------|----------|
| 1 | pad_1 | -0.7000, 0.0480 | -0.7002, 0.0482 | 0.3 mm |
| 2 | pad_2 | -0.7000, 0.1680 | -0.7000, 0.1680 | 0.0 mm |
| 3 | pad_3 | -0.7000, 0.2880 | -0.7000, 0.2880 | 0.0 mm |
| 4 | pad_1 | -0.7014, 0.0388 | -0.7015, 0.0388 | 0.0 mm |
| 5 | pad_2 | -0.6905, 0.1565 | -0.6905, 0.1566 | 0.1 mm |

Cycle 4's first pick attempt tripped the IK-branch guard (contorted approach branch) and
the vertical-lift retreat exhausted its 10 Cartesian attempts, but the ready-pose
fallback + pick retry recovered fully — the cycle still placed at 0.0 mm. This matches
the known ~97% single-attempt / ~100%-with-retries MoveIt behavior from pre-upgrade runs.

## Verdict

**PASS.** The full sim stack works on Isaac Sim 6.0.1 / env_isaacsim6 in both insertion
modes: dp 5/5 placed (mean XY error ≈ 6.0 mm, matching known DP bias), moveit 5/5 placed
(≤ 0.3 mm). Clean auto-exit + record flush verified twice. The only regression found was
environmental (conda py3.12 vs ROS python nodes), fixed in `scripts/edi_ros_env.sh`.
Baseline established for the cuMotion comparison (Iteration 8).
