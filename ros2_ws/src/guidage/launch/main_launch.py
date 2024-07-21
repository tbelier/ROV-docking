from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.actions
import os
import launch
from ament_index_python import get_package_share_directory
import datetime

now = datetime.datetime.now()
date_str = now.strftime("%Y-%m-%d_%H:%M")

def generate_launch_description():
	pkg_share = get_package_share_directory("guidage")
	rviz_config_file = os.path.join(pkg_share, "config", "config.rviz")
	
	# node_RawData = Node(
	# 	package='subsonus_pkg', # nom du package
	# 	executable='nodeRawdata_surface', # nom de l'executable
	# 	name='nodeRawdata_surface', # nom du node lors du lancement
	# )

	node_RawData_ROV = Node(
		package='subsonus_pkg', # nom du package
		executable='nodeRawdata_ROV', # nom de l'executable
		name='nodeRawdata_ROV', # nom du node lors du lancement
	)

	node_TrackPacket = Node(
		package='subsonus_pkg', # nom du package
		executable='nodeTrackPacket', # nom de l'executable
		name='nodeTrackPacket', # nom du node lors du lancement
	)
	
	node_Control = Node(
            package='control',
            executable='transmitter_node',
            name='transmitter_node'
    )
	
	node_Joy = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
    )
	
	Node_rviz = Node(
        package="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        executable="rviz2",
    )

	Node_vision = Node(
        package='nicheVision',
        executable='nicheVisionNode',
        name='nicheVisionNode',
	)
	
	Node_image = Node(
        package='bluerov_ros_camera',
        executable='bluerov_ros_cameraNode',
        name='bluerov_ros_cameraNode',
    )
	
	node_Guidage = Node(
		package='guidage', # nom du package
		executable='nodeGuidage', # nom de l'executable
		name='nodeGuidage', # nom du node lors du lancement
	)
	node_localisation = Node(
		package='localisation', # nom du package
		executable='node_localisation', # nom de l'executable
		name='localisation', # nom du node lors du lancement
	)

	# retour de la fonction avec la liste des nodes à lancer
	return LaunchDescription([
		# Node Kalman
		node_localisation,
		# Nodes USBL
		#node_RawData,
		node_RawData_ROV,
		node_TrackPacket,
		# Nodes Contrôle
		node_Control,
		node_Joy,
		Node_rviz,
		# Nodes vision
		Node_image,
		Node_vision,
		# Node Guidage
		node_Guidage,
		launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'record', '/sensor/yaw_speed','/kalman/cmd_vel', '/bluerov_ros_camera/image_raw','/vision/pose', '/real/cmd_vel', 'subsonus/track_packet', 'subsonus/track_packet_filtered', 'subsonus/raw_sensors_packet_ROV','sensor/pressure'],output='screen') 
		#launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'record', '/sensor/yaw_speed','/kalman/cmd_vel', '/vision/pose', '/real/cmd_vel', 'subsonus/track_packet', 'subsonus/track_packet_filtered', 'subsonus/raw_sensors_packet_ROV','sensor/pressure'],output='screen') 
	])