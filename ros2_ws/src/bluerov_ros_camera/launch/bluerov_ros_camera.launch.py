import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='bluerov_ros_camera',
            executable='bluerov_ros_cameraNode',
            name='bluerov_ros_cameraNode',
            output='screen',
        ),
    ])