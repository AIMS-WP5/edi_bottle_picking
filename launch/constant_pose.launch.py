from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    edi_parameters = {
        "use_sim_time": True
    }

    # Start the actual move_group node/action server
    constant_pose_node = Node(
        package="edi_bottle_picking",
        executable="constant_pose",
        output="screen",
        parameters=[
            edi_parameters,
        ],
    )

    return LaunchDescription([constant_pose_node])
