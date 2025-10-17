import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class SnakeLocomotion(Node):
    def __init__(self):
        super().__init__('snake_locomotion')
        # Joint names as defined in URDF
        self.joint_names = [f'joint{i}' for i in range(1,35)]
        self.client = rclpy.action.ActionClient(self, FollowJointTrajectory,
                                                '/joint_trajectory_controller/follow_joint_trajectory')
        # Wait for action server
        self.client.wait_for_server()
        self.timer = self.create_timer(0.2, self.publish_gait)  # 5 Hz

    def publish_gait(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        A = 0.5  # amplitude (rad)
        omega = 0.5  # frequency (Hz)
        phase = 0.3  # phase shift per joint
        # Compute target positions
        positions = [A * np.sin(2*np.pi*omega*t + i*phase) for i in range(len(self.joint_names))]
        # Build trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1  # 1 second to reach
        traj.points = [point]
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        self.client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SnakeLocomotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
