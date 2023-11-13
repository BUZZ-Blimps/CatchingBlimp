import os
from glob import glob
from setuptools import setup,find_packages

package_name = 'track_ros2'

setup(
    name='ROS2 Tracker for Blimps',
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='corelab',
    maintainer_email='swampblimps@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_ros2_node = track_ros2.yoloAndDeepSort_ros2:main'
        ],
    },
)
