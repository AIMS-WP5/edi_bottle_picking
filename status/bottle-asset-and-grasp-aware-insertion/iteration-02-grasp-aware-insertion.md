# Iteration 2 — Grasp-aware insertion (enables physical grip-in-place end-to-end)

Date: 2026-07-15
Repos touched: `edi_isaacsim` (in-hand transform publication), `edi_bottle_picking`
(grasp-aware insertion + canonical pick orientation + plumbing, this doc).
Plan: `plan.md` (NEXT_STEPS item 2). All features default-OFF; legacy behaviour untouched.

## What was built

1. **Measured in-hand transform, Isaac → ROS** (`edi_isaacsim`): the bottle-in-wrist_3
   transform computed at the FixedJoint bond (`rel`) is now stored per attach and
   published every frame on **`grasp_in_hand`** (PoseStamped; frame_id `wrist_3_link`
   while bonded, sentinel `"none"` otherwise, so stale transforms can never leak across
   cycles). New OmniGraph publisher `EDIInHandPub`; accessor `get_attached_in_hand()`;
   attach log now includes `rel_q_wxyz`.
2. **Grasp-aware insert pose derivation** (`edi_bottle_picking`,
   `compute_grasp_aware_insert_candidates`): desired BOTTLE pose = upright (local +Z up)
   at `socket + grasp_aware_bottle_offset_xyz` (default `[0,0,0.078]` — exactly what the
   calibrated fixed-pose numbers imply: offset_z 0.09 − grip_offset 0.012; the 0.015
   offset_x is the wrist→bottle overhang, which cancels in bottle terms). EE target =
   `T_world_bottle ∘ inverse(T_ee_bottle)` with `T_ee_bottle` from `grasp_in_hand` via
   TF. The bottle's spin about world-Z is a free DOF (axis-symmetric): closed-form
   closest-to-calibrated spin φ* = 2·atan2(B,A), plus ±90°/180° **spin candidates**
   tried best-first through the existing seeded-IK + GUARD2 descent gauntlet.
   Relaxed IK branch cap in grasp-aware mode (3.5 vs 2.0 rad) — mirrored postures
   legitimately need ~2.3–2.6 rad; GUARD2 remains the arbiter.
3. **Canonical pick orientation** (the decisive fix, user-directed): the legacy pick
   verticalized the published bottle quaternion, so bottle roll (±20° settle) and flip
   content leaked into the wrist yaw — every grasp had a different gripper-to-bottle
   relationship (fine under the seat snap, fatal for grip-in-place). In grasp-aware
   mode the pick orientation is rebuilt from the bottle's long-axis AZIMUTH alone:
   yaw = azimuth(neck) + 90° over the top-down base Rx(180°). Reproduces the known-good
   nominal case exactly; flipped bottles get their 180° yaw above the box (free DOF)
   instead of demanding a mirrored wrist at the socket; roll (the physically irrelevant
   DOF) is excluded by construction.
4. **Plumbing**: config keys `grasp_aware_insertion` (default false),
   `in_hand_pose_topic`, `grasp_aware_bottle_offset_xyz`; launch arg
   `grasp_aware_insertion`; bringup flag `--grasp-aware true|false`.

## Findings the hard way (all captured in code comments)

- **Seat-mode equivalence (regression anchor)**: with the canonical seat, the derived
  EE target reproduces the fixed calibrated pose to **0.0 mm / 0.1°** (logged per
  insertion); placement 0.03–0.08 mm over 3 cycles — the whole chain (bond transform →
  TF → free-spin solve) is exact.
- **Mirrored grasps physically collide**: before the canonical pick fix, flipped/rolled
  bottles produced in-hand orientations needing ~135–170° wrist reorientation. All spin
  candidates failed the collision-checked descent (fractions 0.2–0.5) — and a
  collisions-off trial proved that verdict PHYSICAL: the executed mirrored descent hit
  the pad with the asymmetric camera-arm holder, aborted mid-descent, left the bottle
  on the rim (z 1.068) and wedged the arm for all later cycles. Descent collision
  checking is therefore kept ON (config note added); mirror-class candidates fail
  cleanly up front, arm safe, following cycles unaffected (verified).
- The failure→fix ladder: 15.9 mm lying-flat (iteration-07 baseline) → clean guarded
  failures (spin candidates + relaxed cap + honest collision verdict) → **5/5 sub-mm**
  (canonical pick).

## Verification (Isaac 6.0.1, fresh scenes, moveit mode, OMPL, pad-adj 0)

| run | config | result |
|-----|--------|--------|
| A | seat mode + `--grasp-aware true`, 3 cycles | 3/3, 0.03–0.08 mm, derived≡fixed (0.0 mm/0.1°) |
| B pre-fix | `--grip-in-place` + grasp-aware, 5 cycles | canonical-class 0.01–0.52 mm; mirror-class fail cleanly |
| **B final** | `--grip-in-place` + grasp-aware + canonical pick, 5 cycles (GUI) | **5/5 placed, XY 0.02/0.05/0.14/0.11/0.03 mm (mean 0.07), rest z 0.9899–0.9901, all upright; every insertion a uniform 3.7° delta** |

Records: `edi_isaacsim/dptarget-data-20260715-{093222,102440}.json`.
`--grip-in-place` + `--grasp-aware true` now matches the seat-snap baseline (0.06 mm)
with physical suction end-to-end — the seat hack is no longer load-bearing.

## Follow-ups

- Failed insertions still release the bottle at height (pre-existing; noted iter-01).
- The 5 cm attached proxy box + solid-pad model make the checker conservative for
  large-spin candidates; a measured-transform-accurate attached object (iteration 3
  gives the sphere machinery for cuMotion; the MoveIt planning-scene analogue remains
  open) would let mirror-class grasps be re-evaluated honestly — moot now that the
  canonical pick prevents them, but relevant for future non-canonical grasp sources.
- Real-robot: the canonical pick assumes the vision pipeline's bottle quaternion has a
  meaningful long-axis direction; verify against the real `best_grasp` publisher.
