from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package='mnt_fontanilles').find('mnt_fontanilles')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    # Gazebo Server
    gzserver_launch_file = os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
    world_file = os.path.join(pkg_share, "worlds", "fontanilles.world")
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_file),
        launch_arguments={
            "world": world_file,
            "verbose": "false",
            "pause": LaunchConfiguration("paused")
        }.items()
    )
    
    # Gazebo Client
    gzclient_launch_file = os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch_file),
        condition=IfCondition(LaunchConfiguration("gui"))
    )
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        )

    return LaunchDescription([
        DeclareLaunchArgument(name="gui", default_value="true"),
        DeclareLaunchArgument(name="paused", default_value="false"),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        gzserver_launch,
        gzclient_launch,
        rviz_node
    ])
