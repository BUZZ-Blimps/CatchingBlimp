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
            executable='point_cloud_node',
            name='point_cloud_node',
            namespace=LaunchConfiguration('namespace'),
            # Remap outputs to the correct namespace
            remappings=[
                ('BurnCreamBlimp/targets', ['/', LaunchConfiguration('namespace'), '/targets']),
                ('BurnCreamBlimp/points2', ['/', LaunchConfiguration('namespace'), '/points2']),
                ('BurnCreamBlimp/pixels', ['/', LaunchConfiguration('namespace'), '/pixels']),
                ('BurnCreamBlimp/avoidance', ['/', LaunchConfiguration('namespace'), '/avoidance']),
                ('BurnCreamBlimp/left/image_rect_color', ['/', LaunchConfiguration('namespace'), '/left/image_rect_color']),
                ('BurnCreamBlimp/left/camera_info', ['/', LaunchConfiguration('namespace'), '/left/camera_info']),
                ('BurnCreamBlimp/right/camera_info', ['/', LaunchConfiguration('namespace'), '/right/camera_info']),
                ('BurnCreamBlimp/disparity', ['/', LaunchConfiguration('namespace'), '/disparity']),
                ('BurnCreamBlimp/bounding_box', ['/', LaunchConfiguration('namespace'), '/bounding_box']),
            ],
        ),
    ])