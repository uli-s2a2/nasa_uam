import os

from ament_index_python.packages import get_package_share_directory
import launch

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg
import launch.events

def generate_launch_description():
    px4_bridge_dir = get_package_share_directory('px4_bridge')
    launch_dir = os.path.join(px4_bridge_dir, 'launch')

    params_file = os.path.join(px4_bridge_dir, 'params', 'sitl_params.yaml')


    mapping_node = Node(
        package='mapping',
        name='static_obstacle_advertiser',
        executable='obstacle_advertiser',
        output='screen',
        parameters=[params_file])

    control_node = Node(
        package='control',
        executable='qlearning_controller',
        output='screen',
        parameters=[params_file])

    navigator_node = LifecycleNode(
        name="navigator",
        namespace='',
        package='navigator',
        executable='navigator_main',
        output='screen',
        parameters=[params_file])

    planner_node = LifecycleNode(
        name="planner_server",
        namespace='',
        package='planner',
        executable='planner_server',
        output='screen',
        parameters=[params_file])

    visualization_node = Node(
        package='visualization',
        executable='visualization',
        output='screen',
        parameters=[params_file])

    px4_bridge_node = Node(
        package='px4_bridge',
        name='px4_bridge',
        executable='px4_bridge',
        output='screen',
        parameters=[params_file])

    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(px4_bridge_dir, 'rviz', 'config_file.rviz')])

    navigator_configure_trans_event = EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(navigator_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    planner_configure_trans_event = EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(planner_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    register_event_handler_for_navigator_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=navigator_node, goal_state='inactive',
            entities=[
                # Change State event
                launch.actions.EmitEvent( event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matchers.matches_action(navigator_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    register_event_handler_for_planner_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=planner_node, goal_state='inactive',
            entities=[
                # Change State event
                launch.actions.EmitEvent( event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matchers.matches_action(planner_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    ld = LaunchDescription()

    ld.add_action(mapping_node)
    ld.add_action(control_node)
    ld.add_action(navigator_node)
    ld.add_action(planner_node)
    ld.add_action(visualization_node)
    ld.add_action(px4_bridge_node)
    ld.add_action(register_event_handler_for_navigator_inactive_state)
    ld.add_action(register_event_handler_for_planner_inactive_state)
    ld.add_action(navigator_configure_trans_event)
    ld.add_action(planner_configure_trans_event)
    ld.add_action(rviz2_node)

    return ld