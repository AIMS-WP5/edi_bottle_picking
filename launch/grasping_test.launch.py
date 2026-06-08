from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    run_dp_switchover = LaunchConfiguration("run_dp_switchover")

    return LaunchDescription([
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
            }],
        ),
    ])
