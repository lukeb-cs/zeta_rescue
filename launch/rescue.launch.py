from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_aruco'),
                
                'launch',
                'aruco_recognition.launch.py'  # change if your launch file has a different name
             )
         )
        
    )

    return LaunchDescription([
        aruco_launch,
        Node(
            package='zeta_rescue',
            executable='zeta_node',
            name='zeta_node',
        ),
        Node(
            package='zeta_rescue',
            executable='nav_node',
            name='nav_node'
        )
    ])
