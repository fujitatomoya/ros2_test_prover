#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'param',
            default_value=TextSubstitution(text='abc'),
            description='String param'
        ),
        Node(
            package='prover_rclpy',
            executable='ros2cli_862',
            name='mccdaq_daq',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'param': LaunchConfiguration('param'),
            }],
        ),
        ])