from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pose_set = LaunchConfiguration("pose_set")
    use_sim_time = LaunchConfiguration("use_sim_time")
    run_dp_switchover = LaunchConfiguration("run_dp_switchover")

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
            description="Use Isaac Sim /clock as the time source. Also puts grasping_test "
                        "into simulation mode: it skips the UR IO grasp check (no digital IO "
                        "in sim). Leave false for the real robot.",
        ),
        DeclareLaunchArgument(
            "run_dp_switchover",
            default_value="true",
            description="Run the MoveIt->real-time (DP/PyTorch) controller switchover during "
                        "pick (joint_trajectory_controller <-> forward_velocity_controller + "
                        "dp_exec_start signal). Set false for model-less runs.",
        ),
        Node(
            package="edi_bottle_picking",
            executable="grasping_test",
            output="screen",
            parameters=[{
                # LaunchConfiguration is a string; coerce to the node parameter types.
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "run_dp_switchover": ParameterValue(run_dp_switchover, value_type=bool),
                "pose_set": ParameterValue(pose_set, value_type=str),
            }],
        ),
    ])
