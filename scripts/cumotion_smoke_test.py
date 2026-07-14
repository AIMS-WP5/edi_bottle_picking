#!/usr/bin/env python3
"""Smoke-test the standalone cumotion_planner_node against the live sim stack.

Sends two MoveGroup goals straight to the `cumotion/move_group` action server (bypassing
move_group entirely -- this is plan-only, nothing executes):

  1. a joint-space goal: the current configuration nudged by a small delta;
  2. a pose goal for virtual_ee_link: the current EE pose lifted +5 cm in z.

Prerequisites: Isaac playing + the control stack up (joint_states flowing), ONE
cumotion_planner_node running (edi_moveit_config cumotion_planner.launch.py).

Usage:  cumotion_smoke_test.py [--repeat N]
Exit 0 iff every goal returns error_code SUCCESS (val=1) with a non-empty trajectory.
"""
import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint,
                             MotionPlanRequest, OrientationConstraint, PositionConstraint)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class CumotionSmokeTest(Node):
    def __init__(self):
        super().__init__("cumotion_smoke_test")
        self.client = ActionClient(self, MoveGroup, "cumotion/move_group")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _js_cb(self, msg):
        if all(j in msg.name for j in UR_JOINTS):
            self.joint_state = msg

    def wait_ready(self, timeout=180.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.joint_state is not None and self.client.server_is_ready():
                return True
            if not self.client.server_is_ready():
                self.client.wait_for_server(timeout_sec=1.0)
        return False

    def current_q(self):
        m = dict(zip(self.joint_state.name, self.joint_state.position))
        return [m[j] for j in UR_JOINTS]

    def send(self, label, goal):
        t0 = time.time()
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print(f"[{label}] goal rejected or send timeout")
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=120.0)
        if rfut.result() is None:
            print(f"[{label}] result timeout")
            return False
        res = rfut.result().result
        traj = res.planned_trajectory.joint_trajectory
        npts = len(traj.points)
        dur = (traj.points[-1].time_from_start.sec
               + traj.points[-1].time_from_start.nanosec * 1e-9) if npts else 0.0
        ok = res.error_code.val == 1 and npts > 0
        print(f"[{label}] error_code={res.error_code.val} points={npts} "
              f"traj_duration={dur:.2f}s planning_time={res.planning_time:.3f}s "
              f"wall={time.time()-t0:.2f}s -> {'PASS' if ok else 'FAIL'}")
        return ok

    def joint_goal(self, dq):
        q = self.current_q()
        req = MotionPlanRequest()
        req.group_name = "ur_manipulator"
        goal_c = Constraints()
        for name, qi, d in zip(UR_JOINTS, q, dq):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = qi + d
            jc.tolerance_above = jc.tolerance_below = 0.001
            jc.weight = 1.0
            goal_c.joint_constraints.append(jc)
        req.goal_constraints = [goal_c]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = True
        return goal


    def pose_goal(self, dz):
        # current virtual_ee_link pose from TF, lifted dz in base_link z
        t = self.tf_buffer.lookup_transform("base_link", "virtual_ee_link", rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=5.0))
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z + dz
        pose.orientation = t.transform.rotation

        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "virtual_ee_link"
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.002]
        bv = BoundingVolume()
        bv.primitives = [prim]
        bv.primitive_poses = [pose]
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "virtual_ee_link"
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0

        goal_c = Constraints()
        goal_c.position_constraints = [pc]
        goal_c.orientation_constraints = [oc]
        req = MotionPlanRequest()
        req.group_name = "ur_manipulator"
        req.goal_constraints = [goal_c]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = True
        return goal


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repeat", type=int, default=1)
    args = ap.parse_args()

    rclpy.init()
    node = CumotionSmokeTest()
    if not node.wait_ready():
        print("FAIL: no /joint_states or cumotion/move_group server (is the stack + "
              "cumotion_planner.launch.py up?)")
        sys.exit(2)
    print(f"ready: joint_states flowing, cumotion server up; q={['%.3f' % v for v in node.current_q()]}")

    ok = True
    for i in range(args.repeat):
        ok &= node.send(f"joint-goal #{i+1}",
                        node.joint_goal([0.15, 0.05, -0.1, 0.05, 0.1, 0.2]))
        ok &= node.send(f"pose-goal(+5cm z) #{i+1}", node.pose_goal(0.05))
        ok &= node.send(f"joint-goal-return #{i+1}",
                        node.joint_goal([0.0] * 6))
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
