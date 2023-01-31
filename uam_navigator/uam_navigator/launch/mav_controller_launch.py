import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # params_file_path = os.path.join(get_package_share_directory(
    #     'uam_navigator'), 'config', 'params.yaml')

    ld = LaunchDescription()

    mav_controller_node = Node(
        package='uam_navigator',
        executable='mav_controller_node',
        output='screen',
    )

    ld.add_action(mav_controller_node)
    return ld
