#!/usr/bin/env python3
"""Vacuum-gripper command bridge (ROS side).

Exposes a simple on/off interface for the (simulated) vacuum gripper and relays
the state to Isaac Sim as a std_msgs/Bool on /isaac_vacuum_grip:

    true  -> suction ON  (Isaac snaps the bottle to the gripper each sim step)
    false -> suction OFF (Isaac releases it; the bottle falls under gravity)

Command it with a std_srvs/SetBool service (synchronous -- good for sequencing a
pick/place), or just publish to /isaac_vacuum_grip directly for quick tests.

The current state is re-published on a timer so a late-joining / re-discovered
Isaac subscriber always converges to the latest command regardless of QoS.

Parameters:
    topic            (str)   topic relayed to Isaac           [/isaac_vacuum_grip]
    service          (str)   SetBool service to command grip  [/vacuum_gripper/command]
    publish_rate_hz  (float) heartbeat re-publish rate        [10.0]
    initial_state    (bool)  state at startup (false=released)[false]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class VacuumGripperBridge(Node):
    def __init__(self):
        super().__init__("vacuum_gripper_bridge")

        self.declare_parameter("topic", "/isaac_vacuum_grip")
        self.declare_parameter("service", "/vacuum_gripper/command")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("initial_state", False)  # start released

        topic = self.get_parameter("topic").value
        service = self.get_parameter("service").value
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._state = bool(self.get_parameter("initial_state").value)

        self._pub = self.create_publisher(Bool, topic, 1)
        self._srv = self.create_service(SetBool, service, self._on_set)
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"vacuum gripper bridge: call '{service}' (SetBool) -> Bool on '{topic}' "
            f"(initial: {'GRIP' if self._state else 'RELEASE'})"
        )
        self._publish()  # announce initial state immediately

    def _on_set(self, request, response):
        self._state = bool(request.data)
        self._publish()
        response.message = "GRIP (suction ON)" if self._state else "RELEASE (suction OFF)"
        response.success = True
        self.get_logger().info(f"vacuum -> {response.message}")
        return response

    def _publish(self):
        self._pub.publish(Bool(data=self._state))


def main(args=None):
    rclpy.init(args=args)
    node = VacuumGripperBridge()
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
