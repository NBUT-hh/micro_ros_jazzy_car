from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xuegecar_voice_nav',
            executable='goto_node',
            name='goto_node',
            output='screen',
        ),
    ])
