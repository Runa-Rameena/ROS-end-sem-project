from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('snake_robot')
    
    # Include the Gazebo launch file
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'spawn_snake_gazebo.launch.py')
        )
    )
    
    # Launch the snake controller node
    controller_node = Node(
        package='snake_robot',
        executable='python3',
        name='snake_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
        prefix=['python3 ' + os.path.join(pkg_dir, 'snake_robot', 'snake_controller.py')]
    )
    
    return LaunchDescription([
        spawn_launch,
        controller_node
    ])