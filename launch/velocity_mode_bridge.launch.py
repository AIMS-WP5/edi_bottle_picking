from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topic = LaunchConfiguration("topic")
    service = LaunchConfiguration("service")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    initial_state = LaunchConfiguration("initial_state")

    return LaunchDescription([
        DeclareLaunchArgument(
            "topic", default_value="/isaac_velocity_control",
            description="std_msgs/Bool topic relayed to Isaac Sim "
                        "(true=velocity-control gains kp=0, false=position-control gains).",
        ),
        DeclareLaunchArgument(
            "service", default_value="/velocity_control_mode/command",
            description="std_srvs/SetBool service used to set the drive mode.",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz", default_value="10.0",
            description="Heartbeat rate at which the current drive mode is re-published.",
        ),
        DeclareLaunchArgument(
            "initial_state", default_value="false",
            description="Drive mode at startup (false = position control).",
        ),
        Node(
            package="edi_bottle_picking",
            executable="velocity_mode_bridge.py",
            name="velocity_mode_bridge",
            output="screen",
            parameters=[{
                "topic": topic,
                "service": service,
                # LaunchConfiguration values arrive as strings; coerce to the
                # parameter types the node declares, or it rejects the override.
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "initial_state": ParameterValue(initial_state, value_type=bool),
            }],
        ),
    ])
