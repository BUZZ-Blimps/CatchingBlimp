from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blimp_telemetry',
            executable='blimp_telemetry_node',
            name='burn_cream',
            parameters=[
                {'blimpname': 'burn_cream_blimp'}
            ]
        ),
    ])