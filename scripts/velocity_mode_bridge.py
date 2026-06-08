#!/usr/bin/env python3
"""Velocity-control mode bridge (ROS side).

Tells Isaac Sim which joint-drive gains to use, so the robot obeys the controller
that is currently active over the topic_based_ros2_control bridge:

    true  -> velocity-control gains (kp=0, kd high): the velocity commands streamed
             by forward_velocity_controller (e.g. the DP/PyTorch model) drive the arm.
    false -> position-control gains (USD defaults): joint_trajectory_controller's
             position commands hold/track the arm (the default MoveIt path).

Why this is needed: TopicBasedSystem always publishes BOTH a position and a velocity
array on /isaac_joint_commands (both command interfaces are declared in the xacro), and
Isaac's Articulation Controller applies both targets every cycle.  With nonzero stiffness
the stale position target fights the velocity command; zeroing stiffness (kp=0) makes the
velocity command win.  So flip this to true at the same moment you switch to
forward_velocity_controller, and back to false when you return to
joint_trajectory_controller.

Command it with a std_srvs/SetBool service (synchronous -- good for sequencing the
switchover), or just publish to /isaac_velocity_control directly for quick tests.

The current state is re-published on a timer so a late-joining / re-discovered Isaac
subscriber always converges to the latest command regardless of QoS.

Parameters:
    topic            (str)   topic relayed to Isaac              [/isaac_velocity_control]
    service          (str)   SetBool service to set the mode     [/velocity_control_mode/command]
    publish_rate_hz  (float) heartbeat re-publish rate           [10.0]
    initial_state    (bool)  state at startup (false=position)   [false]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class VelocityModeBridge(Node):
    def __init__(self):
        super().__init__("velocity_mode_bridge")

        self.declare_parameter("topic", "/isaac_velocity_control")
        self.declare_parameter("service", "/velocity_control_mode/command")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("initial_state", False)  # start in position control

        topic = self.get_parameter("topic").value
        service = self.get_parameter("service").value
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._state = bool(self.get_parameter("initial_state").value)

        self._pub = self.create_publisher(Bool, topic, 1)
        self._srv = self.create_service(SetBool, service, self._on_set)
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"velocity mode bridge: call '{service}' (SetBool) -> Bool on '{topic}' "
            f"(initial: {'VELOCITY' if self._state else 'POSITION'} control)"
        )
        self._publish()  # announce initial state immediately

    def _on_set(self, request, response):
        self._state = bool(request.data)
        self._publish()
        response.message = (
            "VELOCITY control (kp=0)" if self._state else "POSITION control (USD gains)"
        )
        response.success = True
        self.get_logger().info(f"drive mode -> {response.message}")
        return response

    def _publish(self):
        self._pub.publish(Bool(data=self._state))


def main(args=None):
    rclpy.init(args=args)
    node = VelocityModeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
