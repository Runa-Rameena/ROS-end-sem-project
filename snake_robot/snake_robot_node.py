#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import numpy as np
import math
import time

class SnakeRobotNode(Node):
    def __init__(self):
        super().__init__('snake_robot_node')
        
        self.get_logger().info('Initializing snake robot node...')
        
        # Parameters
        self.declare_parameter('num_joints', 33)
        self.declare_parameter('amplitude', 1.0)  # Increased amplitude for more visible motion
        self.declare_parameter('frequency', 2.0)  # Increased frequency for faster motion
        self.declare_parameter('phase_offset', math.pi/3)  # Changed phase for better wave propagation
        self.declare_parameter('spatial_frequency', 2.0)  # Increased for more waves along body
        
        self.num_joints = self.get_parameter('num_joints').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        self.phase_offset = self.get_parameter('phase_offset').value
        self.spatial_frequency = self.get_parameter('spatial_frequency').value
        
        self.get_logger().info(f'Parameters loaded: joints={self.num_joints}, amplitude={self.amplitude}, frequency={self.frequency}')
        
        # Joint names
        self.joint_names = [f'joint{i}' for i in range(1, self.num_joints + 1)]
        
        # Action client for trajectory control
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Command subscriber
        self.command_sub = self.create_subscription(
            String,
            'snake_command',
            self.command_callback,
            10
        )
        
        # Movement state
        self.movement_state = 'stop'
        self.start_time = self.get_clock().now()
        
        # Create timer for trajectory updates
        self.timer = self.create_timer(0.1, self.update_trajectory)  # 10Hz
        
        self.get_logger().info("Snake robot node initialized")

    def command_callback(self, msg):
        command = msg.data.lower()
        old_state = self.movement_state
        
        if command in ['forward', 'w']:
            self.movement_state = 'forward'
        elif command in ['backward', 's']:
            self.movement_state = 'backward'
        elif command in ['stop', 'space']:
            self.movement_state = 'stop'
            
        if old_state != self.movement_state:
            self.start_time = self.get_clock().now()
            self.get_logger().info(f'Switching to {self.movement_state} movement')

    def _send_trajectory(self, positions):
        """Helper method to create and send trajectory messages."""
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        # Create points for smooth interpolation
        num_points = 5  # Number of interpolation points
        for i in range(num_points):
            point = JointTrajectoryPoint()
            # Interpolate between current and target positions
            point.positions = positions
            point.velocities = [0.0] * len(positions)
            point.accelerations = [0.0] * len(positions)
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int((i + 1) * 0.1 * 1e9)  # Spread over 500ms
            traj.points.append(point)
            
        # Create and send goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        self.trajectory_client.send_goal_async(goal_msg)
        
    def update_trajectory(self):
        if not self.trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Trajectory action server not available')
            return

        if self.movement_state == 'stop':
            # Send zero positions to stop movement
            positions = [0.0] * self.num_joints
            self._send_trajectory(positions)
            return

        current_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        positions = []
        
        direction = 1 if self.movement_state == 'forward' else -1
        base_offset = 0.2  # Base offset to ensure joints move around a non-zero position
        
        # Calculate positions for each joint with improved wave pattern
        for i in range(self.num_joints):
            # Progressive phase shift along the body
            phase = direction * (i * self.phase_offset)
            
            # Main sinusoidal motion with temporal and spatial components
            main_wave = self.amplitude * math.sin(
                2 * math.pi * self.frequency * current_time + 
                self.spatial_frequency * phase
            )
            
            # Add secondary wave for more natural motion
            secondary_wave = 0.3 * self.amplitude * math.sin(
                4 * math.pi * self.frequency * current_time + 
                2 * self.spatial_frequency * phase
            )
            
            # Combine waves and add base offset for alternating joints
            angle = main_wave + secondary_wave
            if i % 2 == 0:
                angle += base_offset
            else:
                angle -= base_offset
                
            positions.append(angle)

        # Send trajectory message with multiple points for smoother motion
        self._send_trajectory(positions)

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = SnakeRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.try_shutdown()
        except:
            pass

if __name__ == '__main__':
    main()