# Iteration 3 — cuMotion robot assets: stripped-URDF generator + EDI UR5e XRDF

Date: 2026-07-14
Repos touched: `edi_ur` (edi_moveit_config), branch `feature/cumotion-integration`;
`edi_bottle_picking` (this status doc).

## New files (edi_moveit_config)

- **`scripts/generate_cumotion_urdf.py`** (installed to `lib/edi_moveit_config/`):
  xacro-expands `edi_ur5e.urdf.xacro` (name=ur, ur_type=ur5e, gripper_type=vacuum, same
  args as the live robot_description) and strips `ros2_control`, `transmission`,
  `gazebo`, `visual`, `collision` elements (23 removed) → kinematics-only URDF at
  `~/.ros/cumotion/edi_ur5e_cumotion.urdf` (path printed on stdout for launch capture).
  Stripping visual/collision follows the robo-codegen-edi precedent and removes
  `package://` mesh URIs that yourdfpy can't resolve; collision comes from XRDF spheres.
  Fails loudly if `virtual_ee_link` is missing or ros2_control survives.
- **`config/cumotion/edi_ur5e.xrdf`** (XRDF format 1.0, NVIDIA layout):
  - `tool_frames: ["virtual_ee_link"]` — the frame `manipulator_interface` targets.
  - Arm collision spheres from NVIDIA's shipped `ur5e.xrdf` (27 spheres, Lula-derived);
    accel 12 / jerk 500 (conservative, per robo-codegen findings).
  - `default_joint_positions` = EDI default posture (matches Isaac scene initial pose).
  - **NEW tool spheres** measured from the collision STLs: 4 on `holder` (camera arm,
    +x to 0.129) + 6 on `ecbpi_link` (vacuum body along +z to 0.26). The last ~2 cm at
    the suction tip is deliberately uncovered so pre-grasp poses beside the bottle don't
    false-positive (grasp close + insertion descent are Cartesian; cuMotion never plans
    them). Self-collision ignores added for the rigidly-attached wrist/tool pairs.
- CMakeLists: install rule for the script (config/ dir already installed).

## Verification

- Generated URDF: 0 `ros2_control`/`visual`/`collision` elements; `virtual_ee_link`
  present at tool0 + (0, 0, 0.305), rpy (0, 0, 3.140); holder→ecbpi chain intact.
- **FK cross-check** (scratchpad `verify_cumotion_fk.py`): curobo `CudaRobotModel` built
  via `convert_xrdf_to_curobo(ContentPath(xrdf, urdf))` reports ee link
  `virtual_ee_link`; FK at the EDI default posture matches independent yourdfpy FK to
  **0.0003 mm** (base_link → virtual_ee_link = [0.4753, -0.6316, 0.2541]).
  37 collision spheres active (27 arm + 10 tool).

## Notes

- Installed curobo API differs from some docs: `convert_xrdf_to_curobo` takes a
  `ContentPath` (from `curobo.types.file_path`), not positional paths.
- XRDF format_version 1.0 chosen (robo-codegen's 2.0 would only warn, but 1.0 matches
  the shipped parser's expectations exactly).

## Verdict

**PASS.** cuMotion-ready robot description with correct EDI kinematics and tool
collision coverage. Next (Iteration 4): launch file for cumotion_planner_node + live
smoke test against the sim.
