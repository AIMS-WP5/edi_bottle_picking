#!/usr/bin/env python3
"""
Regression test for the box-wall vs vacuum-gripper collision that made the
conveyor_feeding pick fail on bottles sitting near a box wall.

Background: the pick's Cartesian approach to a bottle near the +x box wall stalled
at the final waypoint because the gripper link 'ecbpi_link' grazed the TOP of the
'bottle_box' wall (contact ~z=1.028, overlap z in [1.004, 1.08]). The fix lowers the
wall height. This test sweeps the wall height and reports, for the *exact* problematic
grasp configuration, the tallest wall that is still collision-free -- so we can keep
the walls as tall as possible while clearing the gripper.

It also (optionally) runs the full "command robot to above_box_1, plan a Cartesian
path to the bottle approach pose" check at a chosen height.

Needs move_group up (e.g. bringup_sim_stack.sh --no-pick). Run:
    python3 test_box_wall_clearance.py            # sweep
    python3 test_box_wall_clearance.py --height 0.14   # single full Cartesian-plan check
"""
import sys
import math
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity, ApplyPlanningScene, GetCartesianPath, GetPositionFK
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion


def _slerp(q0, q1, a):
    """q as (x,y,z,w)."""
    d = sum(x * y for x, y in zip(q0, q1))
    if d < 0:
        q1 = [-x for x in q1]; d = -d
    if d > 0.9995:
        r = [x + a * (y - x) for x, y in zip(q0, q1)]
    else:
        th = math.acos(d); s = math.sin(th)
        c0 = math.sin((1 - a) * th) / s; c1 = math.sin(a * th) / s
        r = [c0 * x + c1 * y for x, y in zip(q0, q1)]
    n = math.sqrt(sum(x * x for x in r))
    return [x / n for x in r]


def interpolate_waypoints(start, goal, num_intermediate=3):
    """Mirror ManipulatorInterface::interpolateWaypoints: num_intermediate+2 poses,
    linear position + slerp orientation, INCLUDING the start pose as waypoint 0."""
    wps = []
    qs = (start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w)
    qg = (goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w)
    for i in range(num_intermediate + 2):
        a = i / (num_intermediate + 1)
        p = Pose()
        p.position.x = start.position.x * (1 - a) + goal.position.x * a
        p.position.y = start.position.y * (1 - a) + goal.position.y * a
        p.position.z = start.position.z * (1 - a) + goal.position.z * a
        qi = _slerp(qs, qg, a)
        p.orientation = Quaternion(x=qi[0], y=qi[1], z=qi[2], w=qi[3])
        wps.append(p)
    return wps

# ---- the exact problematic geometry (from the failing iteration-88 diagnostic) ----
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# IK config that reaches the bottle-approach pose (Cartesian endpoint that collided)
GRASP_CONFIG = [0.0286, -1.3429, 1.7052, -1.9327, -1.5710, -0.7295]
# the bottle-approach Cartesian target (pick_pose[0]) and start pose 'above_box_1'
APPROACH_XYZ = (-0.2890, -0.5000, 1.0025)
APPROACH_QUAT = (-0.0128, 0.0, 0.0, 0.9999)   # (x,y,z,w)
ABOVE_BOX_1 = [-0.2091, -1.1771, 1.0022, -1.4191, -1.5069, 0.0239]

# bottle_box geometry (mirrors add_collision_box(pose=(-0.45,-0.50,0.93), 0.40,0.30,H,0.015))
BOX_POSE = (-0.45, -0.50, 0.93)
BOX_X, BOX_Y, BOX_TH = 0.40, 0.30, 0.015
BOTTLE_TOP_Z = 0.984   # tops of the lying bottles (for context)


def _prim(co, dims, off):
    p = SolidPrimitive(); p.type = SolidPrimitive.BOX; p.dimensions = list(dims)
    pose = Pose(); pose.position = Point(x=off[0], y=off[1], z=off[2]); pose.orientation.w = 1.0
    co.primitives.append(p); co.primitive_poses.append(pose)


def make_box(height):
    """bottle_box CollisionObject (floor + 4 walls) at the given wall height."""
    co = CollisionObject()
    co.header.frame_id = "world"; co.id = "bottle_box"; co.operation = CollisionObject.ADD
    co.pose.position = Point(x=BOX_POSE[0], y=BOX_POSE[1], z=BOX_POSE[2])
    co.pose.orientation.w = 1.0
    x, y, th, z = BOX_X, BOX_Y, BOX_TH, height
    _prim(co, (x, y, th),  (0.0, 0.0, th / 2))                  # floor
    _prim(co, (x, th, z),  (0.0,  y / 2 - th / 2, z / 2))       # wall +y
    _prim(co, (x, th, z),  (0.0, -(y / 2 - th / 2), z / 2))     # wall -y
    _prim(co, (th, y, z),  ( x / 2 - th / 2, 0.0, z / 2))       # wall +x  (the one we hit)
    _prim(co, (th, y, z),  (-(x / 2 - th / 2), 0.0, z / 2))     # wall -x
    return co


class WallClearanceTest(Node):
    def __init__(self):
        super().__init__("box_wall_clearance_test")
        self.apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid = self.create_client(GetStateValidity, "/check_state_validity")
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        for c, n in [(self.apply, "/apply_planning_scene"), (self.valid, "/check_state_validity")]:
            while not c.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"waiting for {n} ...")

    def _call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        return fut.result()

    def set_box(self, height):
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(); req.scene.is_diff = True
        req.scene.world.collision_objects.append(make_box(height))
        self._call(self.apply, req)

    def state_collides(self, config):
        req = GetStateValidity.Request()
        req.group_name = "ur_manipulator"
        js = JointState(); js.name = ARM_JOINTS; js.position = config
        req.robot_state = RobotState(); req.robot_state.joint_state = js
        res = self._call(self.valid, req)
        if res is None:
            return None, "<no response>"
        if res.valid:
            return False, "collision-free"
        pairs = [f"{c.contact_body_1}<->{c.contact_body_2}(d={c.depth*1000:.1f}mm)" for c in res.contacts]
        return True, "; ".join(pairs) if pairs else "invalid (no contacts reported)"

    def _eef_pose(self, config, link="virtual_ee_link"):
        req = GetPositionFK.Request()
        req.header.frame_id = "world"; req.fk_link_names = [link]
        js = JointState(); js.name = ARM_JOINTS; js.position = config
        req.robot_state = RobotState(); req.robot_state.joint_state = js
        res = self._call(self.fk, req)
        return res.pose_stamped[0].pose if res and res.pose_stamped else None

    def cartesian_fraction(self, start_config):
        """Faithful 'command to above_box_1, plan Cartesian to the bottle' check:
        same start EEF pose + 5-waypoint interpolation as ManipulatorInterface::cartesian_goal."""
        start_pose = self._eef_pose(start_config)
        if start_pose is None:
            return None
        goal = Pose()
        goal.position = Point(x=APPROACH_XYZ[0], y=APPROACH_XYZ[1], z=APPROACH_XYZ[2])
        goal.orientation = Quaternion(x=APPROACH_QUAT[0], y=APPROACH_QUAT[1],
                                      z=APPROACH_QUAT[2], w=APPROACH_QUAT[3])
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"; req.group_name = "ur_manipulator"
        req.link_name = "virtual_ee_link"
        js = JointState(); js.name = ARM_JOINTS; js.position = start_config
        req.start_state = RobotState(); req.start_state.joint_state = js
        req.waypoints = interpolate_waypoints(start_pose, goal, 3)
        req.max_step = 0.005; req.jump_threshold = 0.0; req.avoid_collisions = True
        res = self._call(self.cart, req)
        return None if res is None else res.fraction


def main():
    rclpy.init()
    t = WallClearanceTest()
    args = sys.argv[1:]

    if "--height" in args:
        h = float(args[args.index("--height") + 1])
        t.set_box(h)
        coll, why = t.state_collides(GRASP_CONFIG)
        frac = t.cartesian_fraction(ABOVE_BOX_1)
        print(f"\nwall height {h:.3f} (top z={0.93+h:.3f}):")
        print(f"  grasp-config collision : {'COLLIDES' if coll else 'clear'}  [{why}]")
        print(f"  Cartesian plan fraction: {frac:.4f}  ({'PASS' if frac and frac >= 0.999 else 'FAIL'})")
    else:
        print(f"\nSweeping wall height (bottle tops ~z={BOTTLE_TOP_Z}). "
              f"grasp config {GRASP_CONFIG}\n")
        print(f"  {'height':>7} {'wall_top_z':>11}  result")
        max_clear = None
        h = 0.150
        while h >= 0.049:
            t.set_box(round(h, 3))
            coll, why = t.state_collides(GRASP_CONFIG)
            top = 0.93 + h
            tag = "COLLIDES" if coll else "clear"
            print(f"  {h:7.3f} {top:11.3f}  {tag:9s} {why}")
            if not coll and max_clear is None:
                max_clear = round(h, 3)
            h -= 0.005
        print(f"\n  => tallest collision-free wall height: "
              f"{max_clear if max_clear is not None else 'NONE in range'} "
              f"(top z={0.93+max_clear:.3f})" if max_clear else "  => none clear in range")

    t.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
