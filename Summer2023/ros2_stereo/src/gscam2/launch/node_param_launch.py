"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Your camera namespace
# camera_name = 'BurnCreamBlimp'

# Location of configuration directory
config_dir = os.path.join(get_package_share_directory('gscam2'), 'cfg')
print(config_dir)

# Parameters file
params_file = os.path.join(config_dir, 'params.yaml')
print(params_file)

# Camera calibration file
camera_config = 'file://' + os.path.join(config_dir, 'my_camera.ini')
print(camera_config)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='my_namespace',
            description='Namespace for the node'
        ),
        Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            name='gscam_publisher',
            namespace=LaunchConfiguration('camera_name'),
            parameters=[
                {'gscam_config': LaunchConfiguration('gscam_config')},
                # Some parameters from a yaml file
                params_file,
                # A few more parameters
                {
                    'camera_name': LaunchConfiguration('camera_name'),  # Camera Name
                    'camera_info_url': camera_config,  # Camera calibration information
                    
                },
            ],
            # Remap outputs to the correct namespace
            remappings=[
                (['/', LaunchConfiguration('camera_name'), '/image_raw'], ['/', LaunchConfiguration('camera_name'), '/sync/image_raw']),
                (['/', LaunchConfiguration('camera_name'), '/camera_info'], ['/', LaunchConfiguration('camera_name'), '/camera_info']),
            ],
        ),
    ])