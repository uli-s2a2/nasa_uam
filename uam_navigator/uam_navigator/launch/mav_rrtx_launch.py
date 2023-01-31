import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # params_file_path = os.path.join(get_package_share_directory(
    #     'uam_navigator'), 'config', 'params.yaml')

    ld = LaunchDescription()

    mav_rrtx_node = Node(
        package='uam_navigator',
        executable='mav_rrtx_node',
        output='screen',
        # parameters=[params_file_path],
    )

    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('uam_navigator'), 'config', 'config_file.rviz')]
    )

    ld.add_action(mav_rrtx_node)
    ld.add_action(rviz2_node)
    return ld
