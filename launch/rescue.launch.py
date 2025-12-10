from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node'
        ),
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
