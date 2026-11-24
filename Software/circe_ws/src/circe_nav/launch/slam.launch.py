#!/usr/bin/env python3
"""slam_toolbox (async) on /scan_filtered; owns map -> odom.

    ros2 launch circe_nav slam.launch.py [use_rviz:=true]

On Jazzy slam_toolbox is a lifecycle node, so it is configured/activated here
(or by lifecycle_manager_localization when use_lifecycle_manager:=true).
`ros2 lifecycle get /slam_toolbox` showing "unconfigured" means nothing activated it.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = Path(get_package_share_directory('circe_nav'))
    default_params = pkg / 'config' / 'mapper_params_circe.yaml'
    default_rviz = pkg / 'rviz' / 'slam.rviz'
    assert default_params.is_file(), f'missing {default_params}'

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        # Must match the top-level key in the params file.
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_lifecycle_manager': use_lifecycle_manager,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    # unconfigured -> inactive
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    # inactive -> active, once configuring has actually finished
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time. True only under Gazebo.'),
        DeclareLaunchArgument(
            'params_file', default_value=str(default_params),
            description='slam_toolbox mapper parameters'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Open RViz with the SLAM layout'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate slam_toolbox automatically. Without '
                        'this it sits in the unconfigured state and publishes '
                        'nothing. Ignored when use_lifecycle_manager is true.'),
        DeclareLaunchArgument(
            'use_lifecycle_manager', default_value='false',
            description='Let an external nav2 lifecycle manager drive the node '
                        'instead of the autostart events here.'),

        slam,
        # Register the activate handler before configuring, or the transition can be missed.
        activate_event,
        configure_event,

        # RViz's Nav2 panel looks for exactly this name to show "Localization: active".
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['slam_toolbox'],
                # 0 stops the manager tearing slam_toolbox down after activation.
                'bond_timeout': 0.0,
            }],
            condition=IfCondition(LaunchConfiguration('use_lifecycle_manager')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            output='screen',
            arguments=['-d', str(default_rviz)],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
