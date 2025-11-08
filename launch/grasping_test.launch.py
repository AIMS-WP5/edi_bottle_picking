from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    edi_parameters = {
        "use_sim_time": True
    }

    # Start the actual move_group node/action server
    grasping_test_node = Node(
        package="edi_bottle_picking",
        executable="grasping_test",
        output="screen",
        parameters=[
            edi_parameters,
        ],
    )

    return LaunchDescription([grasping_test_node])
