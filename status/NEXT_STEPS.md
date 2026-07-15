# Status after bottle-asset + grasp-aware-insertion session (2026-07-15) & next steps

Session docs: `bottle-asset-and-grasp-aware-insertion/` (plan + iterations 01–02).
Prior cuMotion integration: `cumotion-initial-integration/` (complete, 2026-07-14).
Branch `feature/cumotion-integration`, all repos, pushed.

## Done this session

1. **Improved bottle asset re-adopted** (`edi_isaacsim d79f401`): robo-codegen
   `bottle_v3.usd` (accurate two-cylinder collision) restored; nominal radius corrected
   to the true 0.0176 m; rolling fixed with per-bottle PhysX angular damping
   (`--bottle-angular-damping`, default 4.0); seat geometry derived from the measured
   TOOL_TIP_LENGTH 0.3013 m (+ radius + 1.1 mm cup stretch = the calibrated 0.320 —
   an A/B proved seat deltas map 1:1 into placement bias, so the total is preserved;
   URDF `virtual_ee_link` stays 0.305). The iteration-08 "~90° asset axis difference"
   was disproved by measurement (meshes byte-identical): it was pre-grasp rolling.
   Verified: 5/5 moveit-mode placements, mean XY 0.06 mm.
2. **Grasp-aware insertion** (`edi_bottle_picking 2d84df1` + `edi_isaacsim bb42a8b`):
   `--grasp-aware true` derives the insertion EE pose from the MEASURED bottle-in-hand
   transform (new `grasp_in_hand` topic, published at the suction bond), with
   closed-form free-spin solve + spin candidates through the seeded-IK/GUARD2 gauntlet,
   and a canonical pick orientation (wrist yaw from bottle-axis azimuth only — flips
   resolved above the box, settle-roll excluded). **`--grip-in-place` now works
   end-to-end**: 5/5 placed, mean XY 0.07 mm, all upright — physical suction with no
   seat snap, matching the seat-hack baseline. Defaults all OFF; legacy byte-identical.

Use: Isaac `--grip-in-place` + `bringup_sim_stack.sh --insertion-mode moveit
--grasp-aware true` (works with `--planner ompl|cumotion`; insertion segment is
OMPL-pinned as before).

## Stopped / descoped (user decision 2026-07-15)

3. **Attached-bottle visibility to cuMotion** — work stopped before implementation.
   Groundwork discovered for a future session (no install needed): the apt
   `cumotion_planner_node` already runs an `UpdateLinkSpheres` action server at
   **`planner_attach_object`** (flattened [x,y,z,r] spheres, `object_link_name:
   "attached_object"`, 100-sphere buffer auto-provisioned for XRDF robots; C++
   typesupport for `isaac_ros_cumotion_interfaces` is installed). The measured
   `grasp_in_hand` transform + the bottle's known cylinders give exact spheres
   analytically — the separately-packaged depth-camera `object_attachment` node is
   NOT required. Sphere frame should be verified via `viz_all_spheres/
   planner_attach_object` before trusting plans.

## Remaining backlog (from the cuMotion session)

4. nvblox/ESDF world for perception-driven obstacles (`read_esdf_world`, off by default).
5. Real-robot validation: cuMotion native jerk-limited timing (`--retime-plans false`)
   with tuned `--time-dilation`; and verify the canonical pick against the real vision
   pipeline's `best_grasp` orientation convention.

## New follow-ups from this session

- Failed insertions release the bottle from height (~1.22 m) — add a gentler recovery
  (lower before release / return to box) before real-robot runs.
- MoveIt planning-scene attached object is still the 5 cm proxy box; a
  measured-transform-accurate shape would make the descent collision checker honest
  for non-canonical grasps (moot for the canonical pick, relevant for future grasp
  sources).
- DP mode still uses the seat snap; grasp-aware currently applies to moveit insertion
  only.
