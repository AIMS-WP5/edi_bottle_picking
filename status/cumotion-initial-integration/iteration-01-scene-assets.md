# Iteration 1 — Fixed-collision scene assets + tool-length reconciliation

Date: 2026-07-14
Repos touched: `edi_isaacsim` (branch `feature/cumotion-integration`);
`edi_bottle_picking` (this status doc).
Plan: `cumotion-integration-plan.md` in this directory.

## Asset port (robo-codegen-edi → edi_isaacsim)

Both repos' `SimEnvs/assets/{madara_pad.usd, bottle_v3.usd}` share the same lineage
(robo-codegen imported them from edi_isaacsim at `c5c3b0b`), so the fixed versions are
drop-in. Ported:

- **`madara_pad.usd`** — robo-codegen `3ee9486` "accurate pad socket collision +
  ground-truthed insert depth": replaces the lumpy convexDecomposition (floor 4 mm
  proud, off-center rests → wedge/press/pad-shove artifacts) with 19 accurate invisible
  collision boxes (socket wall ring apothem 0.0215 m, floor at measured 0.039 m);
  original cooked mass (0.489 kg) pinned.
- **`bottle_v3.usd`** — robo-codegen `b0ef548` bottle collision fix (part of "cleaner
  cortex bottle→socket placement").

The scene (`simplified_ur5_scene.py`) references both by path — no code changes needed.

## Tool-length reconciliation (decision gate)

The plan flagged reconciling robo-codegen's measured suction-tip length
(`TOOL_TIP_LENGTH = 0.3013` m from tool0, verified two independent ways) against
WS_EDI's `virtual_ee_link`. Finding:

- The 0.24045 m offset cited during planning belongs to the **robotiq_2f** gripper
  branch of `edi_ur5e.urdf.xacro` (L118–123). The **vacuum** branch — the one the sim
  stack uses — defines `virtual_ee_link` at **0.305 m** past tool0 (L133–138).
- The robot USD is byte-identical between the two repos
  (`SimEnvs/edi_ur5e_new/edi_ur5e_no_table.usd`, md5 `04c55c3c…`), so robo-codegen's
  0.3013 m measurement applies directly to our scene's geometry.
- **Discrepancy: 3.7 mm** (URDF frame 0.305 vs physical tip 0.3013 — the URDF thinks
  the tip is slightly farther out than it is). This mirrors robo-codegen's own history
  (0.320 → 0.306 → 0.3013); our 0.305 is at their intermediate-correction level.
- **Decision: defer the URDF change.** All `moveit_insert_*` offsets and the grasp
  z-calibration were tuned with 0.305 and the baseline places at ≤0.3 mm, so the 3.7 mm
  is absorbed by calibration. Revisit only if grasp/insert contact issues appear (or
  when recalibrating for cuMotion in Iteration 8). The cuMotion XRDF (Iteration 3) will
  use `virtual_ee_link` as-is from the URDF — internal consistency between MoveIt and
  cuMotion matters more than absolute tip truth.

## Verification: moveit-mode 5-cycle before/after

Before = Iteration 0 moveit run (old assets): 5/5 placed, ≤0.3 mm XY error, but rest
**z-height spread ~12 mm** (0.9821–0.9968) — the lumpy convexDecomposition floor.
After (fixed assets, same Isaac flags `--pad-adj-x 0 --pad-adj-y 0`), records
`edi_isaacsim/dptarget-data-20260714-124729.json`:

| cycle | pad | XY error | rest z |
|-------|-----|----------|--------|
| 1 | pad_1 | 0.1 mm | 0.9900 |
| 2 | pad_2 | 0.2 mm | 0.9899 |
| 3 | pad_3 | 0.0 mm | 0.9899 |
| 4 | pad_1 | 0.1 mm | 0.9901 |
| 5 | pad_2 | 0.0 mm | 0.9900 |

5/5 placed. XY accuracy unchanged (≤0.2 mm), and the **z-height spread collapsed from
~12 mm to 0.2 mm** — the accurate socket-floor collision gives a deterministic rest
depth, exactly what the pad fix was for. One pick IK transient (error -31) on cycle 2
recovered via the standard retry path, as in the baseline.

## Verdict

**PASS.** Fixed-collision assets adopted with no regression and a clear z-consistency
improvement; tool-length change deferred (documented above). Logs archived at
`/tmp/edi_sim_logs/iter1_moveit_archive/`.
