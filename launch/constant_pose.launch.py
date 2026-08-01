from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pose_set = LaunchConfiguration("pose_set")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pose_set",
            default_value="edi",
            description="Which cell's named SRDF poses to drive. 'edi' (default) = the real EDI "
                        "cell's box-approach poses (above_box_2 / near_box), which are equally "
                        "robo-codegen-edi's sim bin. 'isaac' = the legacy edi_isaacsim scene's box "
                        "(above_box_1 / ai_after_pickup / wait_slam) -- edi_isaacsim is being "
                        "DEPRECATED, and bringup_sim_stack.sh passes this to keep that world "
                        "running meanwhile. The two sets put the tool on OPPOSITE SIDES of the "
                        "robot, so the wrong one does not fail cleanly. Overrides the YAML "
                        "'pose_*' values; individual poses can still be overridden with the "
                        "pose_initial / pose_above_box / ... node parameters.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use Isaac Sim /clock as the time source. Also puts constant_pose into "
                        "simulation mode: it skips the UR /set_io vacuum path (no digital IO in "
                        "sim; suction is modelled by the Isaac bridge) and the facade flips the "
                        "Isaac drive gains during the DP segment. Leave false for the real robot.",
        ),
        # Start the actual move_group node/action server
        Node(
            package="edi_bottle_picking",
            executable="constant_pose",
            output="screen",
            parameters=[{
                # LaunchConfiguration is a string; coerce to the node parameter type.
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "pose_set": ParameterValue(pose_set, value_type=str),
            }],
        ),
    ])
