from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    edi_parameters = {
        "use_sim_time": False
    }

    # Start the actual move_group node/action server
    conveyor_feeding_node = Node(
        package="edi_bottle_picking",
        executable="conveyor_feeding",
        output="screen",
        parameters=[
            edi_parameters,
        ],
    )

    return LaunchDescription([conveyor_feeding_node])
