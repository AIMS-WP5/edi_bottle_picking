# Plan — Bottle asset fixups, grasp-aware insertion, attached-bottle visibility

Session: 2026-07-14 (evening, autonomous — user unavailable until 2026-07-15).
Branch: `feature/cumotion-integration` on all five repos.
Scope: NEXT_STEPS items 1–3 (see `../cumotion-initial-integration/NEXT_STEPS.md`), plus
reconciling the robo-codegen-edi measured gripper length (0.3013 m) where it matters.
Status docs: this directory; one `iteration-NN-*.md` per iteration, committed + pushed
with that iteration's code changes in every affected repo.

## Verified facts the plan builds on (from code exploration, 2026-07-14)

Isaac scene (`edi_isaacsim`, `simplified_ur5_scene.py` / `control_omnigraph.py`):
- Bottles spawn from `SimEnvs/assets/bottle_v3.usd` (spawn loop L968–974); nominal
  radius `_BOTTLE_R = 0.0194` (L697) drives row pitch and rest height. No Python-side
  physics material / damping — bottle physics live in the USD.
- `attach_bottle_to_gripper` (L472–559) computes the exact **bottle-in-wrist_3
  transform** (`rel`, L516–533) when the FixedJoint bonds, but only logs it — nothing
  publishes it to ROS. `best_grasp` carries the bottle's live **world** pose.
- Canonical seat (when `GRIP_IN_PLACE=False`): `move_bottle_to_gripper(...,
  Gf.Vec3d(0,0,0.320), bottle_spin_deg=-93.7, grip_offset=0.012)` (L497–501); the
  0.320 m ≈ tool tip (0.3013) + old bottle radius (0.0194) = 0.3207 — a calibrated,
  underived constant. EE frame proxy is `wrist_3_link` (no tool0 prim in the scene).
- `move_bottle_to_gripper` (L313–427) treats **bottle local +X as the long axis**.

robo-codegen-edi (`cumotion` branch; bottle values identical on `main`):
- Improved `bottle_v3.usd` (blob `b6d2bb9`, same bytes as edi_isaacsim commit
  `30086da`'s version, reverted by `228fdc1`): visual-mesh collision disabled, two
  invisible collision cylinders — body **r=0.0176, h=0.121**, neck r=0.010, h=0.018,
  restOffset=0. Visual radius == collision radius = 0.0176 (nominal 0.0194 is the
  stale number). Geometry is **native Z-long**. No rolling friction/damping authored
  anywhere (USD or Python) — the unrealistic rolling is unfixed upstream too.
- `TOOL_TIP_LENGTH = 0.3013` (`ur5e_common.py:32`; tool0 → suction-cup tip along +Z,
  verified two independent ways 2026-07-12). WS_EDI URDF `virtual_ee_link` (vacuum
  branch) = tool0 + **0.305** — 3.7 mm long; all `moveit_insert_*` offsets are
  calibrated around 0.305 (Iteration-1 decision: don't move the frame).

ROS stack (`edi_bottle_picking` / `edi_robot_control`):
- Insertion segment `run_moveit_insert_segment()` (`conveyor_feeding_utils.cpp`
  L699–892) uses the **fixed** `moveit_insert_orientation_xyzw` +
  `moveit_insert_offset_xyz` (config yaml L17/L24) — grasp-independent; this is why
  `--grip-in-place` placed 15.9 mm off with the bottle lying sideways (iteration-07).
- Attach point in `try_pick_bottle()`: `attach_collision_object` → `command_vacuum
  (true)` → `get_grasped_status()` (L376–386). No in-hand-transform subscription
  exists anywhere (grep-confirmed).
- cuMotion: apt install 3.2.x under /opt/ros/humble. The planner node **already runs
  an `UpdateLinkSpheres` action server at `planner_attach_object`**, and for XRDF
  robots auto-adds `extra_collision_spheres: {attached_object: 100}` to the curobo
  model. The separately-packaged `isaac_ros_cumotion_object_attachment` (NOT
  installed; apt needs sudo — unavailable tonight) is only a depth-camera-driven
  client for that same action. C++ typesupport for
  `isaac_ros_cumotion_interfaces/action/UpdateLinkSpheres` is installed.
- `pick`, `ai_start2` handoff, `safe_retreat`, and the whole insert segment are
  OMPL-pinned via `PipelineScope`; cuMotion plans the free-space transfers.

## Iteration 1 — Bottle asset re-adoption + derived seat constants (edi_isaacsim)

Re-adopt the improved bottle and fix the three reasons it was reverted:

1. **Asset**: restore robo-codegen `bottle_v3.usd` (`git revert 228fdc1` or copy the
   blob — identical bytes). Keep the mesh-accurate collision (body r=0.0176).
2. **Nominal diameter**: `_BOTTLE_R` 0.0194 → **0.0176** (collision == visual radius
   of the new asset). Row pitch/rest-z recompute automatically; also update the
   `control_omnigraph.py` displaced-audit tolerance comment (L129).
3. **Rolling friction/damping**: neither USD has any — add it in Python at spawn
   (PhysX rigid-body API on each bottle prim): angular damping + max angular
   velocity, values as module constants with CLI overrides
   (`--bottle-angular-damping`, per the externalize-constants convention). Tune so
   bottles settle quickly but contact behavior (pick/insert) is unchanged.
4. **Axis convention**: the new asset's internal axes differ ~90° about the long
   axis (iteration-08 observation: visible twirl at seat). Inspect both USD versions
   with a pxr script (env_isaacsim6) to measure the actual delta, then compensate in
   the canonical-seat `bottle_spin_deg` (module constant `BOTTLE_SPIN_DEG`, CLI
   override, default adapted −93.7 ± 90 as measured).
5. **Gripper length**: introduce `TOOL_TIP_LENGTH = 0.3013` in the scene and derive
   the seat translation as `TOOL_TIP_LENGTH + _BOTTLE_R (+ press margin)` instead of
   the hardcoded 0.320. Document that URDF `virtual_ee_link` stays at 0.305
   (calibration-preserving; grasp-aware mode makes the insertion independent of it).

Verify: (a) pxr inspection script confirms collision cylinders + axis delta;
(b) headless Isaac + `bringup_sim_stack.sh --insertion-mode moveit --no-debug
--bottle-picking-iterations 5` (Isaac `--pad-adj-x 0 --pad-adj-y 0`): 5/5 placed,
XY mean ≤ ~0.5 mm, rest z ≈ 0.990 (iteration-08 baseline: 0.1 mm), and bottles
visibly settle in the displaced-bottle audit / spawn logs (no runaway rolling).
Rollback: single-repo revert.

## Iteration 2 — Grasp-aware insertion (edi_isaacsim + edi_bottle_picking [+ edi_robot_control if needed])

Goal: insertion wrist pose derived from the **measured** bottle-in-hand transform,
enabling `--grip-in-place` (no seat snap) end-to-end. Optional, default off.

1. **Publish the in-hand transform** (edi_isaacsim): at FixedJoint bond, hand `rel`
   (bottle in wrist_3 frame) to control_omnigraph; publish as `PoseStamped` on new
   topic `grasp_in_hand` (frame_id `wrist_3_link`), latched/republished each frame
   while attached (OmniGraph generic publisher, `:`-joined nested fields). Cleared
   on detach.
2. **Consume it** (edi_bottle_picking): new subscription (config
   `in_hand_pose_topic: "grasp_in_hand"`), new param `grasp_aware_insertion`
   (default **false**; launch arg + bringup flag `--grasp-aware true|false`).
3. **Math** (in `run_moveit_insert_segment`, replacing L739–746 when enabled and a
   fresh in-hand pose exists): with T_ee_bottle = (tf: virtual_ee_link←wrist_3) ∘
   rel, solve the EE target from the desired bottle pose:
   `T_world_ee = T_world_bottle_target ∘ inverse(T_ee_bottle)`.
   Desired bottle pose: long axis (bottle local **+Z**, new asset) vertical,
   bottom at the same depth the calibrated numbers imply, XY on socket + the
   calibrated XY correction; free spin about world-Z chosen to maximize closeness
   to the calibrated `moveit_insert_orientation_xyzw` (keeps IK in the known-good
   basin; NATURAL_INSERT_SEED retry logic unchanged). Descent stays vertical.
   Sanity property: with the canonical seat transform as input, the derived EE
   target must reproduce today's fixed target to ~mm/deg — this is the regression
   anchor and will be asserted in logs.
4. Keep the fixed-orientation path byte-identical when the flag is off or no
   in-hand pose arrived (warn + fall back).

Verify: (a) flag-off 3-cycle moveit regression (placement unchanged);
(b) flag-on with canonical seat: derived target ≈ fixed target in logs, placement
unchanged; (c) Isaac `--grip-in-place` + `--grasp-aware true`: bottle inserted
upright (records status placed, z ≈ 0.990), placement recovers from iteration-07's
15.9 mm / lying-flat failure to low-mm. Planner: ompl first, then one cumotion run.
Rollback: default-off flag.

## Iteration 3 — Attached-bottle visibility to cuMotion (optional, default off)

No new installs: talk directly to the planner node's `planner_attach_object`
(`UpdateLinkSpheres.action`: `attach_object`, `flattened_sphere_arr [x,y,z,r]*N`,
`object_link_name: "attached_object"`).

1. **Client** (edi_bottle_picking): action client + helper that generates bottle
   spheres analytically — body cylinder r=0.0176/h=0.121 → ~5 spheres, neck → 1 —
   placed via the measured in-hand pose (iteration-2 topic; fallback: canonical
   seat transform), expressed in the `attached_object` parent frame (curobo
   convention: the tool frame, `virtual_ee_link`; verified at runtime via the
   `viz_all_spheres/planner_attach_object` topic before trusting plans).
2. **Wiring**: attach after grasp-confirm in `try_pick_bottle`, detach at release
   (both no-ops unless enabled). New param `cumotion_attach_object` (default
   **false**), bringup flag `--cumotion-attach true|false` (warn if used without
   `--planner cumotion`). Action failures log a warning and never block the cycle.
3. **Known risk** (why default off): near-contact start states with bottle spheres
   at above_box/retreat may hard-fail cuMotion starts (3.2 quirk). The OMPL-pinned
   segments are unaffected by design.

Verify: (a) toggle-off = byte-identical behavior; (b) toggle-on cumotion 3-cycle
run completes with sphere attach/detach logged by the planner node each cycle;
(c) sanity: an exaggerated test sphere (scratch script) visibly changes/blocks a
transfer plan, proving the spheres are honored; sphere frame confirmed via the viz
topic. Rollback: default-off flag.

## Wrap-up

Write `status/NEXT_STEPS.md` (top level) with the remaining backlog (nvblox/ESDF,
real-robot validation, any new follow-ups), final placement numbers, and flag
documentation; commit + push all touched repos.

## Risks / notes

- Isaac runs are driven autonomously (headless) for verification; logs under
  /tmp/edi_sim_logs, placement from dptarget-data records in edi_isaacsim.
- Angular-damping values are contact-physics changes — kept out of the pick/insert
  contact path (damping only affects free rolling; verify placement parity).
- The ~90° axis delta sign is measured (not guessed) before changing bottle_spin_deg.
- `attached_object` sphere frame is verified via viz before enabling plans on it.
- Pushes: edi_isaacsim + edi_bottle_picking → GitHub; edi_ur + edi_ai_moveit_servo
  (+ edi_robot_control) → gitlab.edi.lv, as configured per repo.
