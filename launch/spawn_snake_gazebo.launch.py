import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('snake_robot')
    
    # World file path
    world_file_path = os.path.join(pkg_share, 'worlds', 'empty_world.world')
    
    # Load Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file_path],
        output='screen'
    )

    # Robot Description from XACRO
    robot_description_content = Command(
        [
            'xacro',
            ' ',
            os.path.join(pkg_share, 'urdf', 'snake.urdf.xacro'),
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Load controllers config
    controller_config = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'snake',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
    )

    # Load joint state broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Load joint trajectory controller
    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Delay loading of controllers
    load_joint_state_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    load_joint_trajectory_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller],
        )
    )

    return LaunchDescription([
        # Start Gazebo first
        gazebo,
        
        # Start robot state publisher
        robot_state_pub,
        
        # Spawn robot with delay
        TimerAction(
            period=3.0,
            actions=[spawn_entity]
        ),
        
        # Load controllers with event handlers
        load_joint_state_event,
        load_joint_trajectory_event,
    ])