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
        DeclareLaunchArgument(
            'calibration_file',
            default_value='camera1',
            description='Name of the package containing calibration files'
        ),
        DeclareLaunchArgument(
            'camera_ns',
            default_value='BurnCreamBlimp',
            description='Namespace for the camera'
        ),
        launch_ros.actions.Node(
            package='opencv_telemetry',
            executable='split_sync_images',
            namespace=LaunchConfiguration('namespace'),
            name='split_sync_image_node',
            parameters=[
                {'calibration_file': LaunchConfiguration('calibration_file')},
                {'camera_ns': LaunchConfiguration('camera_ns')}
            ],
        ),
  ])
