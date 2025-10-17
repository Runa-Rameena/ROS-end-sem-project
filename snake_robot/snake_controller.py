#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import numpy as np
import time
from std_msgs.msg import String
import subprocess
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
from pathlib import Path
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time

class SnakeController(Node):
    def __init__(self):
        super().__init__('snake_controller')
        self.get_logger().info("Initializing snake controller...")
        
        # Create a callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_joints', 33),
                ('amplitude', 0.5),
                ('frequency', 1.0),
                ('phase_offset', math.pi/4),
                ('spatial_frequency', 1.0)
                ,('spawn_on_start', False),
                ('spawn_x', 2.0),
                ('spawn_y', 0.0),
                ('spawn_z', 0.5),
                ('xacro_path', '')
            ]
        )
        
        # Create a subscriber for commands
        self.command_sub = self.create_subscription(
            String,
            'snake_command',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.num_joints = self.get_parameter('num_joints').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        self.phase_offset = self.get_parameter('phase_offset').value
        self.spatial_frequency = self.get_parameter('spatial_frequency').value
        self.spawn_on_start = self.get_parameter('spawn_on_start').value
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.spawn_z = self.get_parameter('spawn_z').value
        self.xacro_path = self.get_parameter('xacro_path').value
        
        
        # Wait for action server
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Create timer for trajectory generation
        self.timer = self.create_timer(0.1, self.generate_trajectory, callback_group=self.callback_group)
        self.t = 0.0
        self.get_logger().info("Snake controller initialized")
        self.joint_names = [f'joint{i+1}' for i in range(self.num_joints)]
        self.direction = 1.0  # Forward movement by default
        self.is_moving = True

        # Publisher fallback to animate joints when action server is unavailable
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # buffer of previous angles to create a travelling wave along the body
        self.prev_angles = [0.0 for _ in range(self.num_joints)]

        # Optionally spawn model on start
        if self.spawn_on_start:
            self.get_logger().info('spawn_on_start is True: attempting to spawn model')
            self.create_timer(1.0, self._spawn_timer_cb, callback_group=self.callback_group)

    def _spawn_timer_cb(self):
        # single-shot timer: cancel after running
        try:
            if hasattr(self, '_spawn_timer') and self._spawn_timer is not None:
                try:
                    self._spawn_timer.cancel()  # Cancel the timer
                except Exception:
                    pass
                self._spawn_timer = None
            # attempt spawn once
            self.spawn_model()
        finally:
            # cancel this timer by shutting it down: easier to just set flag
            self.spawn_on_start = False

    def generate_trajectory(self):
        if not self.is_moving:
            return

        server_available = self.action_client.wait_for_server(timeout_sec=0.1)

        # Compute head angle and propagate it down the body using a shift buffer
        t_now = self.t
        head_angle = self.amplitude * math.sin(2.0 * math.pi * self.frequency * t_now * self.direction)

        # shift angles down the chain (tail follows head)
        for idx in range(self.num_joints - 1, 0, -1):
            # simple follower: previous segment takes angle of predecessor
            self.prev_angles[idx] = self.prev_angles[idx - 1]
        self.prev_angles[0] = head_angle

        # Optionally apply a spatial offset to create curvature: small per-segment phase
        positions = []
        for j in range(self.num_joints):
            # combine travelling wave buffer with a small static curvature term
            curvature = - j * (self.phase_offset * 0.02) * self.spatial_frequency
            positions.append(self.prev_angles[j] + curvature)

        if server_available:
            # Create trajectory message and send as action goal
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * self.num_joints
            point.accelerations = [0.0] * self.num_joints
            point.time_from_start = Duration(sec=1, nanosec=0)

            trajectory.points = [point]

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            self.action_client.send_goal_async(goal)
        else:
            # Action server not available: publish JointState so a local visualization or other node can animate
            js = JointState()
            now = self.get_clock().now().to_msg()
            js.header.stamp = now
            js.name = self.joint_names
            js.position = positions
            # velocities/efforts left empty
            self.joint_state_pub.publish(js)

        # advance time for next cycle
        self.t += 0.1

    def spawn_model(self):
        # Attempt to build URDF from provided xacro path or from package
        try:
            xacro_path = self.xacro_path
            if not xacro_path:
                # default to package urdf
                pkg_share = Path(__file__).parents[2]
                xacro_path = str(pkg_share / 'urdf' / 'snake.urdf.xacro')

            self.get_logger().info(f'Generating URDF from xacro: {xacro_path}')

            # Try to run xacro to expand macros; if xacro not available or file missing, fall back to raw URDF
            xml = None
            try:
                proc = subprocess.run(['xacro', xacro_path], capture_output=True, text=True, timeout=10)
                if proc.returncode == 0:
                    xml = proc.stdout
                else:
                    self.get_logger().warn(f'xacro returned non-zero: {proc.stderr}')
            except Exception:
                self.get_logger().warn('xacro command failed or not available; attempting to read raw URDF')

            if not xml:
                # Try reading file directly
                try:
                    with open(xacro_path, 'r') as f:
                        xml = f.read()
                except Exception as e:
                    self.get_logger().error(f'Failed to load URDF/XACRO: {e}')
                    # try alt location in package share
                    try:
                        pkg_share = get_package_share_directory('snake_robot')
                        alt = Path(pkg_share) / 'urdf' / 'snake.urdf'
                        with open(alt, 'r') as f:
                            xml = f.read()
                            self.get_logger().info(f'Loaded fallback URDF from {alt}')
                    except Exception as e2:
                        self.get_logger().error(f'Fallback URDF also failed: {e2}')
                        return False

            # Wait for spawn service
            self.get_logger().info('Waiting for /spawn_entity service...')
            client = self.create_client(SpawnEntity, '/spawn_entity')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('/spawn_entity service not available')
                return False

            pose = Pose()
            pose.position.x = float(self.spawn_x)
            pose.position.y = float(self.spawn_y)
            pose.position.z = float(self.spawn_z)

            req = SpawnEntity.Request()
            req.name = 'snake'
            req.xml = xml
            req.robot_namespace = ''
            req.initial_pose = pose
            req.reference_frame = 'world'

            fut = client.call_async(req)
            self.get_logger().info('Called /spawn_entity, waiting for response...')
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done():
                resp = fut.result()
                self.get_logger().info(f"Spawn response success={getattr(resp,'success',False)} message={getattr(resp,'status_message', '')}")
                return getattr(resp, 'success', False)
            else:
                self.get_logger().warn('Spawn service call did not finish in time')
                return False
        except Exception as e:
            self.get_logger().error(f'Exception while spawning: {e}')
            return False
    
    def forward(self):
        self.direction = 1.0
        self.is_moving = True
        self.get_logger().info('Moving forward')
    
    def backward(self):
        self.direction = -1.0
        self.is_moving = True
        self.get_logger().info('Moving backward')
    
    def stop(self):
        self.is_moving = False
        self.get_logger().info('Stopping')
        
    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'forward' or cmd == 'w':
            self.forward()
        elif cmd == 'backward' or cmd == 's':
            self.backward()
        elif cmd == 'stop' or cmd == ' ':
            self.stop()

def main(args=None):
    rclpy.init(args=args)
    snake_controller = SnakeController()
    executor = MultiThreadedExecutor()
    executor.add_node(snake_controller)
    
    try:
        print("Snake controller is running. Send commands to /snake_command topic:")
        print("forward or w - Move forward")
        print("backward or s - Move backward")
        print("stop or space - Stop")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        snake_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()