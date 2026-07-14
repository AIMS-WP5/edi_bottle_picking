# Iteration 2 — GPU/ABI probe + cuMotion install: apt route WORKS on RTX 5090

Date: 2026-07-14
Repos touched: none (system-level install + pip --user only; this status doc).
Plan: `cumotion-integration-plan.md` in this directory.

## Outcome

**The apt install works on the RTX 5090 (Blackwell, sm_120) — no source build needed.**
The plan's headline risk (prebuilt CUDA kernels lacking sm_120) does not materialize
because curobo ships **no prebuilt kernels** in the apt packages at all: it JIT-compiles
its four CUDA extension modules (kinematics_fused, geom, tensor_step, lbfgs_step +
line_search) via torch's cpp_extension loader on first import, targeting the local GPU.
One-time compile ≈ 221 s; afterwards served from `~/.cache/torch_extensions/py310_cu128`
(re-import 3.9 s). The per-import "JIT compiling..." messages are cosmetic.

## What was installed

apt (NVIDIA repo `deb https://isaac.download.nvidia.com/isaac-ros/release-3 jammy
release-3.0`, key at `/usr/share/keyrings/isaac-ros.gpg`,
`/etc/apt/sources.list.d/isaac-ros.list`) — installed by user (sudo; slow due to this
machine's known dkms rebuild issue, unrelated to these packages):

- ros-humble-isaac-ros-cumotion **3.2.7-0jammy** (+ curobo at
  `/opt/ros/humble/lib/python3.10/site-packages/curobo`)
- ros-humble-isaac-ros-cumotion-moveit 3.2.5-0jammy
- ros-humble-isaac-ros-cumotion-robot-description 3.2.5-0jammy (ships `ur5e.xrdf` etc.)
- ros-humble-isaac-ros-cumotion-interfaces / -python-utils 3.2.5, ros-humble-nvblox-msgs
  3.2.5 (auto deps — no vendoring needed)

pip --user (system python 3.10; apt curobo doesn't declare its pip deps):
`warp-lang 1.15.0`, `yourdfpy 0.0.60` (+ trimesh chain).

## Probe results (scratchpad probe_cumotion_gpu.py, system py3.10, conda stripped)

- torch 2.7.1+cu128, CUDA available, device RTX 5090, capability (12,0).
- curobo `MotionGen` on builtin ur5e config: init+warmup 9.1 s,
  **`plan_single_js` success, solve_time 0.026 s**, 32-step trajectory.

## ABI / integration sanity

- `ldd /opt/ros/humble/lib/libisaac_ros_cumotion_moveit.so` — all libraries resolve
  against binary MoveIt 2.5.9 (the GitHub issue #27 ABI mismatch does not apply here:
  the workspace uses binary MoveIt; only moveit_task_constructor is vendored).
- Plugin descriptor confirms class
  `isaac_ros_cumotion_moveit/CumotionPlanner` (base `planning_interface::PlannerManager`).
- `import isaac_ros_cumotion.cumotion_planner` OK (nvblox_msgs present).

## Notes for later iterations

- First cumotion_planner_node start after a torch/CUDA/driver upgrade will re-JIT
  (~4 min); the bringup readiness poll (Iteration 7) must tolerate this.
- Set `TORCH_CUDA_ARCH_LIST=12.0` in the cumotion launch env if the "all archs"
  JIT warning ever becomes a compile-time problem; currently harmless.
- Rollback: `sudo apt-get purge 'ros-humble-isaac-ros-*' ros-humble-nvblox-msgs`,
  remove `/etc/apt/sources.list.d/isaac-ros.list`, `pip uninstall warp-lang yourdfpy`.

## Verdict

**PASS.** cuMotion planning stack fully operational GPU-side; MoveIt plugin loadable.
Next (Iteration 3): edi-specific stripped URDF + XRDF.
