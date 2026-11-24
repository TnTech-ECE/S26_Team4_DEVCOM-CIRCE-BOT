#!/usr/bin/env python3
"""Show the CIRCE robot model in RViz with no hardware attached.

    ros2 launch circe_description display.launch.py gui:=true    # joint sliders
"""

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_path('circe_description')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(pkg / 'urdf' / 'circe_rover.urdf.xacro'),
        description='Absolute path to the robot xacro/urdf to display')
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(pkg / 'rviz' / 'circe_urdf.rviz'),
        description='Absolute path to the RViz config')
    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='false',
        description='Use joint_state_publisher_gui (joint sliders) instead of the '
                    'headless joint_state_publisher')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        model_arg,
        rviz_arg,
        gui_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
