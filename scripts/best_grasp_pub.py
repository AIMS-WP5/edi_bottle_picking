#!/usr/bin/env python3
"""Stand-in for the (absent) vision grasp detector, for exercising the DP loop in sim.

grasping_test reads a grasp pose off the `best_grasp` topic, then UNCONDITIONALLY stamps it
as frame `realsense_455_on_stand` and transforms it to `world` (grasping_test_utils.cpp
~L203-205). So to make the pick land on a chosen WORLD-frame target, we must publish the
pose expressed in the camera frame. This node does that inverse: it takes a world-frame
target (params), looks up world -> realsense_455_on_stand via TF, and republishes the
camera-frame pose on `best_grasp` at a steady rate. The round-trip (here world->cam, then
grasping_test cam->world) returns exactly the target.

Default target = the known-reachable pose hardcoded in constant_pose_utils.cpp (a pose that
has been used for real-robot grasp testing), which is enough to drive the pick through to the
DP segment (in sim the grasp success is faked true regardless). For a meaningful insertion
(Milestone 2), override target_* to a spawned bottle's location.

Run (with the WS_EDI env sourced):
    python3 /tmp/best_grasp_pub.py
    # or override the world target:
    python3 /tmp/best_grasp_pub.py --ros-args -p target_x:=-0.55 -p target_y:=-0.50 -p target_z:=1.0
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # registers do_transform_pose for geometry_msgs
from tf2_geometry_msgs import do_transform_pose


class BestGraspPub(Node):
    def __init__(self):
        super().__init__("best_grasp_pub")

        # World-frame grasp target. Default: directly over the box center (-0.55, -0.50)
        # added by add_box(), at the box floor (BOX_BASE_Z 0.93 + wall 0.015 = 0.945 top)
        # plus ~one bottle width, since the bottles lie sideways inside the box. Tune
        # target_z to sit on the top of a bottle.
        self.declare_parameter("target_x", -0.55)
        self.declare_parameter("target_y", -0.50)
        self.declare_parameter("target_z", 1.0)
        self.declare_parameter("target_qx", -0.999353691)
        self.declare_parameter("target_qy", 0.012828515)
        self.declare_parameter("target_qz", 0.027794998)
        self.declare_parameter("target_qw", -0.00722554)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_frame", "realsense_455_on_stand")
        self.declare_parameter("topic", "best_grasp")
        self.declare_parameter("rate_hz", 5.0)

        g = lambda n: self.get_parameter(n).value
        self.world_frame = g("world_frame")
        self.camera_frame = g("camera_frame")

        self.world_pose = PoseStamped()
        self.world_pose.header.frame_id = self.world_frame
        self.world_pose.pose.position.x = float(g("target_x"))
        self.world_pose.pose.position.y = float(g("target_y"))
        self.world_pose.pose.position.z = float(g("target_z"))
        self.world_pose.pose.orientation.x = float(g("target_qx"))
        self.world_pose.pose.orientation.y = float(g("target_qy"))
        self.world_pose.pose.orientation.z = float(g("target_qz"))
        self.world_pose.pose.orientation.w = float(g("target_qw"))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseStamped, g("topic"), 10)
        self.timer = self.create_timer(1.0 / float(g("rate_hz")), self.tick)
        self._logged_ok = False
        self.get_logger().info(
            f"best_grasp_pub up: world target "
            f"({self.world_pose.pose.position.x:.3f}, {self.world_pose.pose.position.y:.3f}, "
            f"{self.world_pose.pose.position.z:.3f}) in '{self.world_frame}', "
            f"publishing cam-frame pose on '{g('topic')}' (cam '{self.camera_frame}')."
        )

    def tick(self):
        # Need world -> camera to express the world target in the camera frame.
        if not self.tf_buffer.can_transform(
            self.camera_frame, self.world_frame, rclpy.time.Time()
        ):
            self.get_logger().warn(
                f"TF {self.camera_frame} <- {self.world_frame} not available yet; waiting...",
                throttle_duration_sec=2.0,
            )
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, self.world_frame, rclpy.time.Time()
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return

        cam_pose = do_transform_pose(self.world_pose.pose, tf)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.camera_frame  # grasping_test overrides this anyway
        out.pose = cam_pose
        self.pub.publish(out)

        if not self._logged_ok:
            p = cam_pose.position
            self.get_logger().info(
                f"Publishing best_grasp (cam frame): ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"-> grasping_test will transform back to the world target."
            )
            self._logged_ok = True


def main():
    rclpy.init()
    node = BestGraspPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
