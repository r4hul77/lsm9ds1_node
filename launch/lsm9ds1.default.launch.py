from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('lsm9ds1'),
        'config',
        'lsm9ds1_Default.yaml'
        )
    

    lsm9ds1_node = Node(
        package="lsm9ds1",
        executable="lsm9ds1_node",
        parameters=[config]
    )


    ld.add_action(lsm9ds1_node)
    return ld