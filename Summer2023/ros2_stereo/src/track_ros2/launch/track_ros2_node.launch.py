import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='my_namespace',
            description='Namespace for the node'
        ),
        launch_ros.actions.Node(
            package='track_ros2',
            executable='track_ros2_node',
            name='track_ros',
            namespace=LaunchConfiguration('namespace'),
            # Remap outputs to the correct namespace
            remappings=[
                ('bounding_box', ['/', LaunchConfiguration('namespace'), '/bounding_box']),
                ('left/image_raw', ['/', LaunchConfiguration('namespace'), '/left/image_raw']),
            ],
        ),
    ])
