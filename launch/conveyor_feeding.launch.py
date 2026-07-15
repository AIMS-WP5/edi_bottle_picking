from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    debug = LaunchConfiguration("debug")
    iterations = LaunchConfiguration("iterations")
    insertion_mode = LaunchConfiguration("insertion_mode")
    planning_pipeline = LaunchConfiguration("planning_pipeline")
    retime_plans = LaunchConfiguration("retime_plans")
    grasp_aware_insertion = LaunchConfiguration("grasp_aware_insertion")
    pick_depth_flush = LaunchConfiguration("pick_depth_flush")
    moveit_insert_radius_aware = LaunchConfiguration("moveit_insert_radius_aware")

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
        DeclareLaunchArgument(
            "iterations",
            default_value="-1",
            description="Number of full pick/insert cycles to run. -1 (default) keeps the value "
                        "from conveyor_feeding_config.yaml ('iterations'); any value >= 0 overrides "
                        "it. e.g. iterations:=5 for a short test run.",
        ),
        DeclareLaunchArgument(
            "insertion_mode",
            default_value="dp",
            description="Insertion strategy for the socket-insert segment. 'dp' (default) = the "
                        "NN velocity-control segment (ControlModeSwitcher::run_dp_segment); "
                        "'moveit' = comparison test using a MoveIt-planned move above "
                        "socket_center+offset then a straight Cartesian descent (no driver switch, "
                        "no velocity logging). Overrides the YAML 'insertion_mode' value.",
        ),
        DeclareLaunchArgument(
            "planning_pipeline",
            default_value="ompl",
            description="move_group planning pipeline manipulator_interface routes all "
                        "joint-space plans through. 'ompl' (default) = existing behavior; "
                        "'isaac_ros_cumotion' = GPU cuMotion (requires use_cumotion:=true on "
                        "edi_ur_moveit.launch.py and a running cumotion_planner_node). "
                        "Orthogonal to insertion_mode; Cartesian segments are unaffected.",
        ),
        DeclareLaunchArgument(
            "retime_plans",
            default_value="true",
            description="true (default): TOTG re-times every plan (sim-gated 0.2 scaling). "
                        "false: keep the planner's own time parameterization -- with "
                        "planning_pipeline=isaac_ros_cumotion this executes cuMotion's "
                        "jerk-limited timing directly (scale it via the cumotion node's "
                        "time_dilation_factor). A/B knob for the planner comparison.",
        ),
        DeclareLaunchArgument(
            "grasp_aware_insertion",
            default_value="false",
            description="MoveIt insertion mode only. false (default) = fixed calibrated insert "
                        "pose (moveit_insert_orientation_xyzw/offset_xyz). true = derive the "
                        "insert EE pose from the measured bottle-in-hand transform "
                        "(grasp_in_hand, published by Isaac at the suction bond) so the bottle "
                        "ends upright over the socket however it sits in the gripper -- required "
                        "for Isaac's --grip-in-place. Falls back to the fixed pose if no valid "
                        "in-hand transform is available. Overrides the YAML value.",
        ),
        DeclareLaunchArgument(
            "pick_depth_flush",
            default_value="false",
            description="false (default) = pick grasp target unchanged. true = press the grasp "
                        "target 'pick_depth_compliance' m deeper so the rigid cup tip meets the "
                        "bottle surface flush -- intended together with Isaac's "
                        "--best-grasp-at-surface (best_grasp published at the surface). Overrides "
                        "the YAML value.",
        ),
        DeclareLaunchArgument(
            "moveit_insert_radius_aware",
            default_value="false",
            description="MoveIt insertion mode only, and only when grasp_aware_insertion is false. "
                        "false (default) = the literal fixed moveit_insert_offset_xyz pose. true = "
                        "derive the fixed-mode insert pose through the canonical radius-aware "
                        "transform (tracks bottle_radius; reproduces the fixed pose at the baseline "
                        "bottle). Overrides the YAML value.",
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
                "iterations": ParameterValue(iterations, value_type=int),
                "insertion_mode": ParameterValue(insertion_mode, value_type=str),
                "planning_pipeline": ParameterValue(planning_pipeline, value_type=str),
                "retime_plans": ParameterValue(retime_plans, value_type=bool),
                "grasp_aware_insertion": ParameterValue(grasp_aware_insertion, value_type=bool),
                "pick_depth_flush": ParameterValue(pick_depth_flush, value_type=bool),
                "moveit_insert_radius_aware": ParameterValue(moveit_insert_radius_aware, value_type=bool),
            }],
        ),
    ])
