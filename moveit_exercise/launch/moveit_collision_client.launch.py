from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_exercise',
            executable='moveit_test_node',
            name='moveit_test_node',
            output='screen',
        ),
    ])
