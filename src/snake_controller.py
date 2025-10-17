#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import sys

class SnakeController(Node):
    def __init__(self):
        super().__init__('snake_controller')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Wait for action server
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server...')

        # Initialize joint names (33 joints)
        self.joint_names = [f'joint{i}' for i in range(1, 34)]
        self.get_logger().info('Snake controller ready!')

    def create_snake_motion(self, direction='forward', amplitude=0.5, frequency=1.0, duration=5.0):
        """Generate serpentine motion pattern."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        # Parameters for snake-like motion
        num_joints = len(self.joint_names)
        num_points = 50  # Number of waypoints
        phase_offset = np.pi/4  # Phase difference between joints
        
        if direction == 'backward':
            frequency = -frequency

        points = []
        for t in range(num_points):
            time = t * (duration / num_points)
            point = JointTrajectoryPoint()
            
            # Calculate positions for each joint
            positions = []
            for j in range(num_joints):
                # Create a sinusoidal wave that propagates along the snake's body
                angle = amplitude * np.sin(
                    2.0 * np.pi * frequency * time + j * phase_offset
                )
                positions.append(angle)
            
            point.positions = positions
            point.time_from_start = Duration(sec=int(time), nanosec=int((time % 1) * 1e9))
            points.append(point)

        goal_msg.trajectory.points = points
        return goal_msg

    async def move_snake(self, direction='forward'):
        """Send motion command to the snake."""
        self.get_logger().info(f'Moving snake {direction}...')
        goal_msg = self.create_snake_motion(direction=direction)
        
        # Send goal and wait for result
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        await self._send_goal_future
        
        goal_handle = self._send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, executing movement...')
        result = await goal_handle.get_result_async()
        return result

def main():
    rclpy.init()
    snake_controller = SnakeController()
    
    if len(sys.argv) > 1:
        direction = sys.argv[1].lower()
        if direction not in ['forward', 'backward']:
            print("Usage: python3 snake_controller.py [forward|backward]")
            sys.exit(1)
    else:
        direction = 'forward'

    try:
        rclpy.spin_once(snake_controller)
        future = snake_controller.move_snake(direction)
        rclpy.spin_until_future_complete(snake_controller, future)
    except KeyboardInterrupt:
        pass
    finally:
        snake_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()