from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    velocity_controller = Node(
        package='robot_control',
        executable='velocity_controller',
        name='velocity_controller',
        output='screen'
    )

    safety_monitor = Node(
        package='robot_control',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen'
    )

    return LaunchDescription([
        velocity_controller,
        safety_monitor
    ])