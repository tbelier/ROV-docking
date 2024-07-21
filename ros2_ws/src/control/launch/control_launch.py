from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("control")
    rviz_config_file = os.path.join(pkg_share, "config", "config.rviz")
    return LaunchDescription([
        # Control
        Node(
            package='control',
            executable='transmitter_node',
            name='transmitter_node'
        ),
        # Joystick 
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        # Rviz
        Node(
        package="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        executable="rviz2",
    )

    ])
