import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ocam_ros2',
            executable='ocam_ros2',
            name='ocam_ros2',
            output='screen',
            parameters=[
                {"resolution": 2},
                {"frame_rate": 30.0},
                {"exposure": 100},
                {"gain": 50},
                {"wb_blue": 200},
                {"wb_red": 160},
                {"auto_exposure": True},
                {"show_image": True},
            ]
        )
    ])
