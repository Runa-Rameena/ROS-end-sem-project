import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('snake_robot')

    # Robot description from xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'snake.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # Start gzserver (headless) without a world
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # robot_state_publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn entity from topic robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'snake', '-x', '0.0', '-y', '0.0', '-z', '0.5']
    )

    # Start controller node (will publish joint_states if action server missing)
    snake_controller = Node(
        package='snake_robot',
        executable='snake_controller',
        name='snake_controller',
        output='screen',
        parameters=[{'use_sim_time': True, 'spawn_on_start': False}]
    )

    return LaunchDescription([
        gzserver,
        robot_state_pub,
        TimerAction(period=1.0, actions=[spawn_entity]),
        TimerAction(period=1.5, actions=[snake_controller]),
    ])
