# Iteration 3 — Radius-aware, physically-grounded pick & insert (grip-in-place flush)

Date: 2026-07-15
Repos touched: `edi_isaacsim` (best_grasp surface publish); `edi_bottle_picking` (pick-depth
flush + unified radius-aware insert + plumbing, this doc). **`edi_ur` / `edi_robot_control`
untouched.** All features default-OFF; flags-off is byte-identical to iteration 2.
Plan: `~/.claude/plans/mutable-bouncing-gosling.md`.

## Problem

In grip-in-place mode the vacuum cup penetrated the bottle by ~14 mm (vs robo-codegen-edi,
where bottles sit flush). Root cause: the pick descent drives the EE frame to the bottle
**centre** (best_grasp is the centroid; `create_manipulation_poses` adds no radius offset), and
the kinematic FixedJoint freezes that overlap (robo-codegen uses a physics SurfaceGripper that
self-seats flush). The seat-snap used to mask it; grip-in-place exposes it.

## Decisions

- **Keep `virtual_ee_link = 0.305`** (`edi_ur5e.urdf.xacro:137`). It is a deliberate EDI
  hardware value (commit `313f231`, "shorter suction cup" STL) and is shared with the real
  UR5e on `gripper_type:=vacuum` — not sim-only. We do NOT touch the frame. The
  `0.305 − 0.3013 = 3.7 mm` gap to the measured rigid tip is modelled as a **suction-tip
  compliance** constant (generalising `SEAT_CUP_STRETCH`), not a frame error.
- **Split (no double-raise):** radius applied in Isaac `best_grasp`; the ROS pick carries only
  the compliance press.
- **Bottle roll stays off the ROS side** (rotational symmetry): no `canonical_bottle_spin_deg`
  constant; the insert's free world-Z spin search owns orientation.

## What was built

1. **edi_isaacsim `--best-grasp-at-surface`** (`control_omnigraph.py`): publishes best_grasp at
   the bottle top surface (`centroid + _BOTTLE_R`, world +Z) instead of the centroid. Sim-only
   (real robot's grasp point comes from perception).
2. **Pick-depth flush** (`pick_depth_flush` / `--pick-depth-flush`, + `pick_depth_compliance`
   0.0037): presses the grasp target that much deeper so the rigid cup tip meets the surface.
3. **Unified radius-aware insertion** (`moveit_insert_radius_aware` / `--radius-aware-insert`):
   routes fixed-mode insertion through the same `T_we = T_wb · inv(T_eb)` math as grasp-aware,
   with a CANONICAL analytic `T_eb` (`compute_canonical_in_hand_transform`): rotation =
   `inverse(q_cal)`; origin places the EE origin at `(radial_overhang, 0, grip_offset)` in the
   bottle frame, `radial_overhang = bottle_radius − suction_tip_compliance + seat_cup_stretch`.
   `insert_candidates_from_bottle_in_ee` is the shared core; the literal
   `moveit_insert_offset_xyz` remains the final fallback.
4. **Source-of-truth geometry config**: `bottle_radius, grip_offset, suction_tip_length,
   suction_tip_compliance, seat_cup_stretch` (+ the flags). Bringup + launch plumbing mirrors
   `grasp_aware_insertion`.

## Verification (Isaac 6.0.1 headless, `env_isaacsim6`, moveit mode, OMPL, `--pad-adj 0`)

**Standalone (pre-Isaac):** colcon build clean; the canonical-transform algebra reproduces the
fixed calibrated pose to **0.0000 mm / 0.0000°** and the insert overhang tracks radius
(0.0114 / 0.0150 / 0.0194 for r = 0.014 / 0.0176 / 0.022).

| Run | Config | Result |
|-----|--------|--------|
| **1 flags-off** | moveit, 3 cyc | 3/3 placed, XY **0.15/0.08/0.03 mm** (mean ~0.09), z 0.990 — byte-identical baseline |
| **2 radius-aware** | `--radius-aware-insert true`, 3 cyc | 3/3 placed; every cycle **derived≡fixed 0.0 mm / 0.0°** (canonical anchor, live); XY 0.042/0.071/0.072 mm, z 0.990 |
| **3b flush** | `--grip-in-place --best-grasp-at-surface` + `--grasp-aware true --pick-depth-flush true`, 5 cyc | **5/5 placed, XY 0.050/0.032/0.032/0.148/0.014 mm (mean ~0.055), z 0.990, all upright**; pick-depth press fired every cycle |
| **3a baseline** | grip-in-place, no flags, 2 cyc | 2/2 placed; used to A/B the flush geometry |

**best_grasp surface publish:** for the same bottle, `/best_grasp` z went **0.9626 → 0.9802**,
a raise of exactly **0.0176 = one bottle radius**.

**Flush A/B (attach `rel_t` z = bottle centroid in wrist_3; tool0 ≡ wrist_3 origin):**

| Config | `rel_t` z | cup-tip vs surface |
|--------|-----------|--------------------|
| baseline (3a) | **0.3041** | tip penetrates **~14.8 mm** |
| flush (3b) | **0.3179–0.3186** (~0.3182) | tip **at surface**, ~0.7 mm residual |

The flush value ≈ `TOOL_TIP_LENGTH + bottle_radius = 0.3013 + 0.0176 = 0.3189`; Δ vs baseline =
**14.1 mm** — the penetration is eliminated (residual < cup-stretch tolerance).

Records: `edi_isaacsim/dptarget-data-20260715-130610-flush6cyc.json` (runs 1+2),
`…-131949-flush5cyc.json` (run 3b).

## Verdict

**PASS.** Flush achieved in grip-in-place (penetration 14 mm → <1 mm) with placement unchanged
(sub-mm); the unified radius-aware insert reproduces the calibrated pose exactly and tracks the
bottle radius; all flags default-off leave prior behaviour byte-identical. Real-robot frame
(0.305) untouched.

## Notes / follow-ups

- The grip-in-place grasp-aware insert shows a uniform ~12 mm / 3.7° derived-vs-fixed delta —
  expected (the bottle is gripped in place, not seat-snapped); placement stays sub-mm because
  the derivation compensates. Not a regression.
- Real-robot pick-surface offset belongs in the perception pipeline (mirrors the Isaac
  best_grasp change); flagged, not implemented.
- **Axial grasp offset toward the neck (like robo-codegen).** `best_grasp` targets the bottle
  **centre** along the long axis (radial offset only, via `--best-grasp-at-surface`). robo-codegen
  deliberately grasps **~25 mm toward the top** (`madara_bottle` `default_grasp_offset=[0,0,0.025]`
  in bottle-local +Z = neck direction; `robo-codegen-edi/asset_data_utils.py:255`, composed with
  the geometry-derived `grasp_height`). An off-centre / toward-neck grip can carry & insert more
  stably. Clean addition: an **axial** term on `best_grasp` (or a `grasp_axial_offset` config),
  analogous to the radial surface offset; grasp-aware insertion absorbs it automatically (it
  derives from the measured in-hand transform), so no placement retune. Not a bug — a follow-up.
- Different-**length** bottles: `grasp_aware_bottle_offset_xyz[2]` (0.078) is length/socket
  dependent — parametrise on length in a later pass.
- Ops notes: run Isaac in `env_isaacsim6` headless; it traps SIGTERM (use SIGKILL / explicit
  PID, verify via `nvidia-smi --query-compute-apps`); `/isaac_flush_records` fires on a
  **false→true** rising edge (re-flush needs the low pulse first).
- Committed on `feature/cumotion-integration` (edi_isaacsim + edi_bottle_picking); not pushed.
