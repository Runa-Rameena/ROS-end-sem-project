#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sys
import tty
import termios
import threading

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class SnakeController(Node):
    def __init__(self):
        super().__init__('snake_controller')
        
        # Create publisher for joint trajectory controller
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
        
        # Joint names
        self.joint_names = [f'joint{i}' for i in range(1, 34)]
        
        # Snake parameters
        self.amplitude = 0.3  # Maximum angle of the snake's joints (reduced for smoother motion)
        self.frequency = 0.5  # Frequency of the serpentine motion (slower for better coordination)
        self.phase_offset = 0.3  # Phase difference between consecutive joints (reduced for tighter coordination)
        self.wave_length = 1.5  # Wave length of the serpentine motion
        self.direction = 1.0  # 1.0 for forward, -1.0 for backward
        self.is_moving = False
        
        # Create a timer for motion updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        
        # Start keyboard input thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_input)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('Snake controller initialized')
        print("Controls:")
        print("w - Move forward")
        print("s - Move backward")
        print("a - Increase amplitude")
        print("d - Decrease amplitude")
        print("q - Increase frequency")
        print("e - Decrease frequency")
        print("space - Stop")
        print("x - Exit")

    def keyboard_input(self):
        while True:
            key = getch()
            if key == 'w':
                self.direction = 1.0
                self.is_moving = True
                self.get_logger().info('Moving forward')
            elif key == 's':
                self.direction = -1.0
                self.is_moving = True
                self.get_logger().info('Moving backward')
            elif key == ' ':
                self.is_moving = False
                self.get_logger().info('Stopped')
            elif key == 'a':
                self.amplitude = min(1.0, self.amplitude + 0.1)
                self.get_logger().info(f'Amplitude: {self.amplitude}')
            elif key == 'd':
                self.amplitude = max(0.1, self.amplitude - 0.1)
                self.get_logger().info(f'Amplitude: {self.amplitude}')
            elif key == 'q':
                self.frequency = min(2.0, self.frequency + 0.1)
                self.get_logger().info(f'Frequency: {self.frequency}')
            elif key == 'e':
                self.frequency = max(0.1, self.frequency - 0.1)
                self.get_logger().info(f'Frequency: {self.frequency}')
            elif key == 'x':
                self.get_logger().info('Exiting...')
                sys.exit(0)

    def timer_callback(self):
        if not self.is_moving:
            return
            
        # Create trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        # Calculate joint positions for serpentine motion
        current_time = time.time() - self.start_time
        point = JointTrajectoryPoint()
        
        positions = []
        for i in range(len(self.joint_names)):
            # Calculate position along the snake's body (normalized)
            x = i / len(self.joint_names)
            
            # Create a traveling wave that maintains consistent body shape
            # Head (first joint) and next few joints get special treatment for leading the motion
            if i < 3:  # Head and first two joints after head
                # Lead the motion with slightly higher amplitude for the head
                head_factor = 1.2 if i == 0 else 1.1
                angle = head_factor * self.amplitude * np.sin(
                    2.0 * np.pi * (
                        self.frequency * current_time * self.direction -  # Time component
                        x / self.wave_length  # Spatial component
                    )
                )
            else:
                # Body follows with regular sinusoidal motion
                angle = self.amplitude * np.sin(
                    2.0 * np.pi * (
                        self.frequency * current_time * self.direction -  # Time component
                        x / self.wave_length  # Spatial component
                    )
                )
            positions.append(angle)
        
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 seconds
        
        trajectory_msg.points = [point]
        self.trajectory_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    snake_controller = SnakeController()
    try:
        rclpy.spin(snake_controller)
    except KeyboardInterrupt:
        pass
    finally:
        snake_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()