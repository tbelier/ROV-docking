from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.actions
import os
import launch

def generate_launch_description():
	# node_USBL = Node(
	# 	package='subsonus_pkg', # nom du package
	# 	namespace='usbl_1',
	# 	executable='talker', # nom de l'executable
	# 	name='talker', # nom du node lors du lancement
	# )

	node_RawData = Node(
		package='subsonus_pkg', # nom du package
		namespace='usbl_surface',
		executable='nodeRawdata_surface', # nom de l'executable
		name='nodeRawdata_surface', # nom du node lors du lancement
	)

	node_RawData_ROV = Node(
		package='subsonus_pkg', # nom du package
		namespace='usbl_ROV',
		executable='nodeRawdata_ROV', # nom de l'executable
		name='nodeRawdata_ROV', # nom du node lors du lancement
	)

	node_TrackPacket = Node(
		package='subsonus_pkg', # nom du package
		namespace='usbl_surface',
		executable='nodeTrackPacket', # nom de l'executable
		name='nodeTrackPacket', # nom du node lors du lancement
	)

	# retour de la fonction avec la liste des nodes à lancer
	return LaunchDescription([
		# node_USBL,
		node_RawData,
		node_RawData_ROV,
		node_TrackPacket,
		launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a'],output='screen')
	])