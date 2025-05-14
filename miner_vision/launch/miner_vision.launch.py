import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    orbbec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('orbbec_camera'),'launch/'),
             'gemini2L.launch.py']
        )
    )
    detection = Node(
        package='miner_v',
        executable='detection',
        output='screen'
    )
    return LaunchDescription([
        orbbec,
        detection
    ])