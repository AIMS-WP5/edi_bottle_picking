# Iteration 6 — Client-side planner selection + the hybrid cuMotion/OMPL split

Date: 2026-07-14
Repos touched: `edi_robot_control` (**new branch `feature/cumotion-integration`**),
`edi_bottle_picking`, `edi_ur` (XRDF fix), `edi_ai_moveit_servo` (add_pad).

## Planned changes (as per the plan)

- `manipulator_interface`: new ROS params **`planning_pipeline`** (default `ompl`; routes
  all joint-space `plan()` calls via `setPlanningPipelineId`) and **`retime_plans`**
  (default true; false keeps the planner's own timing instead of TOTG — A/B knob for
  Iteration 8). New `set_planning_pipeline()` for scoped overrides.
- `conveyor_feeding.launch.py`: `planning_pipeline` launch arg → node param.

## What live testing forced us to learn (5 root-caused fixes)

Running full pick+insert cycles with `planning_pipeline:=isaac_ros_cumotion` surfaced a
chain of real integration defects — each diagnosed to root cause:

1. **Frame mismatch (edi_ur XRDF)** — isaac_ros_cumotion 3.2 **ignores frame_ids**: it
   interprets mirrored planning-scene object poses AND pose goals in its XRDF base frame.
   Ours was `base_link`, but move_group's planning frame is `world` (base_link at
   z+0.97 m, yaw −135° on the stand) → the table mesh was displaced through the workspace
   (chronic false `INVALID_START_STATE_WORLD_COLLISION`) and pose goals executed ~1 m too
   high (observed on screen). **Fix: `set_base_frame: "world"`** (the URDF root;
   world→base_link is fixed). Verified: curobo FK(home) = [-0.7827, 0.1105, 1.2241] in
   world, matching T_world_base × base-frame FK exactly.
2. **ACM-invisible pad (edi_ai_moveit_servo)** — `madara_pad` was a world collision
   object at the socket target, collision-excluded only via the ACM, which cuMotion does
   not honor → the insertion target itself was an obstacle. **Fix: `pad_as_marker` param
   on add_pad** (default false): publishes an RViz mesh Marker on `/madara_pad_marker`
   instead of a scene object (+ idempotent REMOVE). MoveIt behavior identical (it never
   collided with the pad anyway).
3. **Grasp-target world object (edi_robot_control + edi_bottle_picking)** — the pick
   added a 5 cm "object" box into the world at the grasp pose before planning the
   approach; the approach pose ends at its surface, so cuMotion's buffered IK failed
   every approach (`IK_FAIL`). **Fix: `add_collision_object_simple(..., apply_to_world=
   false)`** — the object is now created only at attach time.
4. **Branch roulette on joint goals** — the cuMotion planner node **converts joint
   targets to an EE pose and re-solves IK**, free to arrive in a different arm basin than
   the seeded-IK config (observed wrapped `wrist_1 ≈ −275°` arrivals whose descent sweeps
   the wrist through the table — user spotted the wrist/table geometry on screen).
5. **Near-contact start states** — curobo validates start states against the mirrored
   world with sphere buffers + activation margin; any config at near-contact (tool at a
   bottle inside the box walls, post-insertion above the pad) hard-fails and wedges all
   recovery planning.

(4) and (5) are structural properties of isaac_ros_cumotion 3.2, not bugs on our side.
**Resolution: hybrid split** (`PipelineScope` RAII in conveyor_feeding_utils) — cuMotion
plans the long free-space moves (wait/above-box/transfers); the **pick sequence,
safe_retreat, and the insertion segment pin themselves to OMPL** (exact joint-branch
control + tolerance of near-contact starts). This mirrors the planned/reactive split
robo-codegen-edi converged on for the same robot.

Additional hardening from the same sessions:
- **GUARD2 now also validates the seeded-IK path** (was fallback-only), and on rejection
  retries IK from a canonical insert-ready seed (`compute_ik_seeded` gained an
  `explicit_seed` arg) before falling back to the guarded global plan. In the verified
  run this retry rescued the insertion in BOTH cycles.
- Ops note: a JTC-activation transient once slammed the sim arm to a garbage pose after
  repeated nonstandard stack restarts (cuMotion then truthfully reported the arm in
  collision — MoveIt's mesh model didn't). Bringup order discipline + verifying
  /joint_states ≈ home before runs avoids it; watch-item, not fixed here.

## Verification (Isaac 6.0.1, fresh scene, verified home start)

**cuMotion 2-cycle run** (`--insertion-mode moveit`, `planning_pipeline:=isaac_ros_cumotion`,
`use_cumotion:=true`, `pad_as_marker:=true`): both cycles complete —
**2/2 placed, XY error 0.1 / 0.0 mm, z = 0.9900 both** (fixed-collision pad determinism).
Pipeline switches logged symmetric (isaac_ros_cumotion ↔ ompl) around the pinned segments.
GUARD2 natural-seed retry engaged in both cycles and produced descendable arrivals.

**OMPL regression** (same code, `planning_pipeline:=ompl`): full cycle completed,
insertion segment success, placement consistent with baseline (sub-mm) — the pick
world-object removal and guard changes do not regress the OMPL path.

## Verdict

**PASS.** cuMotion now services the free-space planning of the bottle-picking task
end-to-end via move_group, with OMPL retained where the 3.2 planner node is structurally
unsuited. Follow-ups tracked to Iteration 8: attached-bottle visibility
(object_attachment), time_dilation/retime A/B, per-segment timing comparison.
