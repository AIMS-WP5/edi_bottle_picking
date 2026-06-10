from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    debug = LaunchConfiguration("debug")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use Isaac Sim /clock as the time source. Also puts conveyor_feeding into "
                        "simulation mode: it skips the UR /set_io vacuum path and the UR IO grasp "
                        "check (no digital IO in sim; suction is modelled by the Isaac bridge) and "
                        "the facade flips the Isaac drive gains during the DP segment. Leave false "
                        "for the real robot.",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="true",
            description="Pause for a 'Next' click in the RViz RvizVisualToolsGui panel between "
                        "each stage of the pick/insert cycle. true = step manually (default, "
                        "matches conveyor_feeding_config.yaml); false = run the full scenario and "
                        "keep cycling unattended. Overrides the YAML 'debug' value. Can also be "
                        "toggled live: ros2 topic pub /conveyor_feeding/debug std_msgs/msg/Bool.",
        ),
        # Start the actual move_group node/action server
        Node(
            package="edi_bottle_picking",
            executable="conveyor_feeding",
            output="screen",
            parameters=[{
                # LaunchConfiguration is a string; coerce to the node parameter type.
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "debug": ParameterValue(debug, value_type=bool),
            }],
        ),
    ])
