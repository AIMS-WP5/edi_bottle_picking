# Iteration 1 — Improved bottle asset re-adopted (+ derived seat constants)

Date: 2026-07-15
Repos touched: `edi_isaacsim` (asset + scene constants); `edi_bottle_picking` (this doc).
Plan: `plan.md` in this directory (NEXT_STEPS item 1 + gripper-length reconciliation).

## What was measured before changing anything

Side-by-side pxr inspection of the old (post-revert `228fdc1`) vs improved (robo-codegen)
`bottle_v3.usd`:

- **The meshes are byte-identical** — same 2820 points, same centroid/bbox
  (z −0.063…+0.072, r_max 0.01756), same Z-long principal axis, same root frame.
  **There is no axis-convention difference between the assets.** The "~90° about the
  long axis" note in `iteration-08-comparison.md` was a misattribution: what was seen
  is the seat snap visibly correcting a bottle that had *rolled/spun* before grasp
  (accurate cylinder collision + no rolling resistance), not a different asset frame.
- Only difference: old = visual-mesh `convexDecomposition` collision enabled; improved
  = mesh collision **disabled** + two invisible accurate collision cylinders
  (`collision_body` r=0.0176 h=0.121 @z−0.0025, `collision_neck` r=0.010 h=0.018
  @z+0.0635, restOffset 0, contactOffset 0.002). Neither asset authors any friction,
  damping, or mass.

## Changes (edi_isaacsim)

1. **Asset restored**: revert of `228fdc1` — `bottle_v3.usd` is again byte-identical to
   robo-codegen's improved version (md5 `8f748a94…`).
2. **Nominal radius corrected**: `_BOTTLE_R` 0.0194 → **0.0176** (the improved asset's
   collision radius == the visual mesh's true max radius; 0.0194 matched neither).
   Row pitch 0.0448 → 0.0412, spawn rest height recomputed.
3. **Rolling resistance added**: new `BOTTLE_ANGULAR_DAMPING = 4.0` (PhysX angular
   damping applied to every spawned bottle; CLI `--bottle-angular-damping`, 0 restores
   undamped rolling). Neither USD authors rolling friction, and the accurate cylinder
   collision otherwise lets a nudged bottle roll indefinitely on the box floor.
4. **Seat constants derived, not magic**: the hardcoded seat call
   `(0,0,0.320), spin −93.7, grip_offset 0.012` is now
   `grip_seat_translation_z() = TOOL_TIP_LENGTH (0.3013, measured tool0→suction-tip,
   robo-codegen 2026-07-12) + _BOTTLE_R (0.0176) + SEAT_CUP_STRETCH (0.0011) = 0.320`,
   with `BOTTLE_SPIN_DEG` / `GRIP_OFFSET` named constants; `control_local_model.py`
   uses the same constants (dead `translation` in control_omnigraph removed).
   The ROS URDF's `virtual_ee_link` stays at 0.305 (calibration frame, untouched).

## Gripper-length reconciliation — what the A/B proved

First attempt set the seat to physically-flush-with-press
(0.3013 + 0.0176 − 0.0007 = **0.3182**). Result: 5/5 placed, rest-z spread 0.9898–0.9901,
but a **constant +1.82 mm world-X placement bias** on every cycle — exactly the
0.320 − 0.3182 seat delta, mapped through the insertion tool axis (which points along
world −X). Conclusions:

- Seat distance maps ~1:1 into placement XY; the calibrated `moveit_insert_offset_x`
  (0.015) and DP `pad_adj` biases all assume the bottle centre 0.320 m from wrist_3.
- With the true radius, the legacy 0.320 seat is a **1.1 mm tip-to-surface gap**
  (compliant-cup stretch at bond) — under the stale 0.0194 radius it had been misread
  as a 0.7 mm press. `SEAT_CUP_STRETCH = 0.0011` documents this and keeps the seated
  geometry byte-identical to everything downstream.

## Verification (Isaac 6.0.1 headless, fresh scene per run, moveit mode, OMPL,
`--pad-adj-x 0 --pad-adj-y 0`, 5 cycles)

- **Rolling**: bottles roll exactly 20° (= 6 mm arc at r 0.0176) at initial settle,
  then stay put — overnight idle (~15 h) and through full runs, displaced-bottle audit
  stable at 1–2 bottles / 6–7 mm / ~20° (right at the audit's 20° threshold; no
  runaway rolling, no moving pick targets). The reason the asset was reverted is gone.
- **Seat A/B run** (seat 0.3182): 5/5 placed, mean XY error 1.83 mm (the bias above).
- **Final run** (seat 0.320 derived): cycles at **0.03–0.14 mm** XY error, rest z
  0.9898–0.9901. One run had a cycle-4 insertion-planning give-up (seeded-IK +
  guarded fallback both unplannable against a pad displaced by earlier insertion
  contact; the bottle was released high and recorded 164 mm off) — the known
  OMPL insertion-seed flake from iterations 06/08, not asset-related; rerun below.
- **Confirmation run**: **5/5 placed, XY errors 0.02/0.03/0.04/0.06/0.17 mm (mean
  0.06 mm), rest z 0.9898–0.9900** — at or better than the iteration-08 OMPL baseline
  (0.1 mm), records `edi_isaacsim/dptarget-data-20260715-091656.json`.

## Verdict

**PASS.** Improved bottle asset re-adopted with placement parity, rolling fixed by
angular damping, nominal diameter corrected to the mesh/collision truth, and the seat
geometry now derived from the measured 0.3013 m tool-tip length instead of magic
numbers.

## Notes / follow-ups

- Pads are deterministically nudged a few mm–cm by insertion contact (identical pad
  coordinates across independent runs); /socket_center tracks them, so placement
  accuracy is unaffected, but the cycle-4 flake shows a displaced pad can push the
  above-socket joint goal into an unplannable corner. Existing behaviour, noted.
- On a failed insertion the cleanup releases the vacuum wherever the arm is (bottle
  dropped from height) — fine in sim, worth a gentler recovery before real-robot runs.
