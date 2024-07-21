import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='bluerov').find('bluerov')
    # default_model_path = os.path.join(pkg_share, 'src/description/auv.urdf')
    default_model_path = os.path.join(pkg_share, 'src/description/bluerov.urdf')
    print(default_model_path)
    
    #world_path = os.path.join(pkg_share, 'world/my_world.sdf')

    # default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    world_launch_share = launch_ros.substitutions.FindPackageShare(package='mnt_fontanilles').find('mnt_fontanilles')
    world_launch_file = os.path.join(world_launch_share, "launch", "mnt_fontanilles.launch.py")
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_file)
    )
    print("bluerov.launch : " + " world launch")
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    print("bluerov.launch : " + " robot state publisher")
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    print("bluerov.launch : " + " joint state")
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    print("bluerov.launch : " + " rviz")

    spawn_entity = launch_ros.actions.Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-entity', 'bluerov', '-topic', 'robot_description', '-x', '1', '-y', '1', '-z', '1'],
      output='screen'
    )
    print("bluerov.launch : " + " spawn entity")
    joycon_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )
    print("bluerov.launch : " + " joycon")
    nodePilotage = launch_ros.actions.Node(
        package='simu_control',
        executable='node_simu_control',
        name='node_simu_control'
    )
    print("bluerov.launch : " + " ")

    Node_vision = launch_ros.actions.Node(
        package='nicheVision',
        executable='nicheVisionNode',
        name='nicheVisionNode',
    )

    Node_image = launch_ros.actions.Node(
        package='bluerov_ros_camera',
        executable='bluerov_ros_cameraNode',
        name='bluerov_ros_cameraNode',
    )
    node_Guidage = launch_ros.actions.Node(
        package='guidage', # nom du package
        executable='nodeGuidage', # nom de l'executable
        name='nodeGuidage', # nom du node lors du lancement
    )

    return launch.LaunchDescription([
        spawn_entity,
        joint_state_publisher_node,

        world_launch,
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),

        # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),

        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time'),

        robot_state_publisher_node,
        # rviz_node,
        nodePilotage,
        joycon_node,

        # Nodes vision
		Node_image,
		Node_vision,
		# Node Guidage
		node_Guidage
    ])