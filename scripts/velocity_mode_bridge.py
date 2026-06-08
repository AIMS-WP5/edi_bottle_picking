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
from sensor_msgs.msg import JointState


class VelocityModeBridge(Node):
    def __init__(self):
        super().__init__("velocity_mode_bridge")

        self.declare_parameter("topic", "/isaac_velocity_control")
        self.declare_parameter("service", "/velocity_control_mode/command")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("initial_state", False)  # start in position control
        # On the switch back to position control, refresh Isaac's position target to the
        # current measured pose (published as a one-shot command) BEFORE restoring stiffness,
        # so the restored kp doesn't yank the arm toward the stale pre-velocity-segment target.
        self.declare_parameter("refresh_position_target", True)
        self.declare_parameter("command_topic", "/isaac_joint_commands")
        self.declare_parameter("joint_states_topic", "/joint_states")

        topic = self.get_parameter("topic").value
        service = self.get_parameter("service").value
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._state = bool(self.get_parameter("initial_state").value)
        self._refresh = bool(self.get_parameter("refresh_position_target").value)

        self._pub = self.create_publisher(Bool, topic, 1)
        self._srv = self.create_service(SetBool, service, self._on_set)
        self._timer = self.create_timer(1.0 / rate, self._publish)

        # Cache the latest /joint_states so we can re-publish the current pose as a hold
        # command on the velocity->position switch.
        self._latest_js = None
        self._cmd_pub = self.create_publisher(
            JointState, self.get_parameter("command_topic").value, 1
        )
        self._js_sub = self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value, self._on_js, 10
        )

        self.get_logger().info(
            f"velocity mode bridge: call '{service}' (SetBool) -> Bool on '{topic}' "
            f"(initial: {'VELOCITY' if self._state else 'POSITION'} control)"
        )
        self._publish()  # announce initial state immediately

    def _on_set(self, request, response):
        new_state = bool(request.data)
        # Switching back to POSITION control: first pin Isaac's position target to the current
        # measured pose so restoring stiffness holds the arm where it is, instead of springing
        # it toward the stale (pre-DP-segment) target.  Publish the hold command BEFORE the
        # gain Bool flips below.
        if self._refresh and (not new_state) and self._latest_js is not None:
            self._publish_hold_command()
        self._state = new_state
        self._publish()
        response.message = (
            "VELOCITY control (kp=0)" if self._state else "POSITION control (explicit gains)"
        )
        response.success = True
        self.get_logger().info(f"drive mode -> {response.message}")
        return response

    def _on_js(self, msg):
        self._latest_js = msg

    def _publish_hold_command(self):
        """Publish the current measured pose on the Isaac joint-command topic as a one-shot
        hold target (zero velocities), so restoring position stiffness does not spring the arm."""
        js = self._latest_js
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(js.name)
        cmd.position = list(js.position)
        cmd.velocity = [0.0] * len(js.position)
        self._cmd_pub.publish(cmd)
        self.get_logger().info(
            f"refreshed Isaac position target to current pose "
            f"({len(cmd.position)} joints) before restoring stiffness"
        )

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
