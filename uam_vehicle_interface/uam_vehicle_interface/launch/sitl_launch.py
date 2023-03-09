import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vehicle_interface_dir = get_package_share_directory('uam_vehicle_interface')
    launch_dir = os.path.join(vehicle_interface_dir, 'launch')

    params_file = os.path.join(vehicle_interface_dir, 'params', 'sitl_params.yaml')


    mapping_node = Node(
        package='uam_mapping',
        name='static_obstacle_advertiser',
        executable='obstacle_advertiser',
        output='screen',
        parameters=[params_file])

    control_node = Node(
        package='uam_control',
        executable='qlearning_controller',
        output='screen',
        parameters=[params_file])

    navigator_node = Node(
        package='uam_navigator',
        executable='navigator_main',
        output='screen',
        parameters=[params_file])

    planner_node = Node(
        package='uam_planner',
        executable='planner_server',
        output='screen',
        parameters=[params_file])

    visualization_node = Node(
        package='uam_visualization',
        executable='visualization',
        output='screen',
        parameters=[params_file])

    vehicle_interface_node = Node(
        package='uam_vehicle_interface',
        executable='px4_interface',
        output='screen',
        parameters=[params_file])


    ld = LaunchDescription()

    ld.add_action(mapping_node)
    ld.add_action(control_node)
    ld.add_action(navigator_node)
    ld.add_action(planner_node)
    ld.add_action(visualization_node)
    ld.add_action(vehicle_interface_node)

    return ld