from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blimp_telemetry',
            executable='blimp_telemetry_node',
            name='burn_cream',
            parameters=[
                {'blimpname': 'BurnCreamBlimp'}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            #arguments = ['0', '0', '0', '0', '0', '0', 'world', 'staticFrame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            #arguments = ['1', '0', '0', '1.57079', '0', '1.57079', 'BurnCreamBlimp', 'BurnCreamBlimp_left_optical_frame']
        ),
    ])
