from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('snake_robot')
    
    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'spawn_snake_gazebo.launch.py')
        ])
    )
    
    # Snake robot controller node
    snake_robot_node = Node(
        package='snake_robot',
        executable='snake_robot_node',
        name='snake_robot_node',
        output='screen',
        parameters=[{
            'num_joints': 33,
            'amplitude': 0.5,
            'frequency': 1.0,
            'phase_offset': 0.785,  # pi/4
            'spatial_frequency': 1.0
        }]
    )
    
    # Snake teleop node
    snake_teleop_node = Node(
        package='snake_robot',
        executable='snake_teleop',
        name='snake_teleop',
        output='screen'
    )
    
    return LaunchDescription([
        # Start Gazebo and spawn robot
        gazebo_launch,
        
        # Start the robot controller after a delay
        TimerAction(
            period=10.0,  # Wait for Gazebo and controllers to be ready
            actions=[snake_robot_node]
        ),
        
        # Start the teleop node
        TimerAction(
            period=12.0,  # Start teleop after controller is ready
            actions=[snake_teleop_node]
        )
    ])