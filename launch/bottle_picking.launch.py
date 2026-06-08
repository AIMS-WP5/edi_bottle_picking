from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    edi_parameters = {
        "use_sim_time": False
    }

    # Start the actual move_group node/action server
    edi_bottle_picking_node = Node(
        package="edi_bottle_picking",
        executable="edi_bottle_picking",
        output="screen",
        parameters=[
            edi_parameters,
        ],
    )

    return LaunchDescription([edi_bottle_picking_node])
