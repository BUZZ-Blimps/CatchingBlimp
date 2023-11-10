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
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            namespace=LaunchConfiguration('namespace'),
            # Remap outputs to the correct namespace
            remappings=[
                ('BurnCreamBlimp/disparity', ['/', LaunchConfiguration('namespace'), '/disparity']),
                ('BurnCreamBlimp/left/image_rect', ['/', LaunchConfiguration('namespace'), '/left/image_rect']),
                ('BurnCreamBlimp/left/camera_info', ['/', LaunchConfiguration('namespace'), '/left/camera_info']),
                ('BurnCreamBlimp/right/image_rect', ['/', LaunchConfiguration('namespace'), '/right/image_rect']),
                ('BurnCreamBlimp/right/camera_info', ['/', LaunchConfiguration('namespace'), '/right/camera_info']),
            ],
        ),
    ])
    