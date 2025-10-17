from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Snake robot node with slower, smoother motion parameters
    snake_robot_node = Node(
        package='snake_robot',
        executable='snake_robot_node',
        name='snake_robot_node',
        output='screen',
        parameters=[{
            'num_joints': 33,
            'amplitude': 0.25,         # Smaller angle swings for gentler curves
            'frequency': 0.3,          # Slower wave frequency (0.3 Hz instead of 0.5)
            'phase_offset': 0.5,       # Reduced phase shift for smoother propagation
            'spatial_frequency': 0.8,  # Fewer waves along the body for fluid motion
        }]
    )   

    return LaunchDescription([
        snake_robot_node
    ])
