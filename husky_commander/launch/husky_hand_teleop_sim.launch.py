import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='husky_commander',  # Replace with your actual package name
            executable='hand_teleop',    # Ensure this matches the installed executable name
            name='hand_teleop_node',
            namespace='a200_0000',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # Adjust if using simulation time
            ],
        )
    ])
