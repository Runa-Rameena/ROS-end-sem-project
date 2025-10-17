#!/usr/bin/env python3
import sys
import select
import tty
import termios

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ESC = '\x1b'

def get_key(nonblocking_timeout=0.1):
    """Non-blocking single-key reader with arrow-key support."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], nonblocking_timeout)
    if not rlist:
        return None
    ch1 = sys.stdin.read(1)
    if ch1 == ESC:
        rlist, _, _ = select.select([sys.stdin], [], [], nonblocking_timeout)
        if rlist:
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                rlist, _, _ = select.select([sys.stdin], [], [], nonblocking_timeout)
                if rlist:
                    ch3 = sys.stdin.read(1)
                    return ESC + '[' + ch3   # Arrow sequence like '\x1b[A'
        return ESC
    return ch1


class SnakeTeleop(Node):
    def __init__(self):
        super().__init__('snake_teleop')
        # Parameterize the command topic; default matches the controller subscriber
        self.declare_parameter('command_topic', 'snakecommand')
        cmd_topic = self.get_parameter('command_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(String, cmd_topic, 10)
        self._orig_settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            "\nSnake Robot Teleop\n"
            "------------------\n"
            "w / ↑ : forward\n"
            "s / ↓ : backward\n"
            "space  : stop\n"
            "q      : quit\n"
            "------------------"
        )

    def publish_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"Sent: {text}")

    def loop(self):
        try:
            if not sys.stdin.isatty():
                self.get_logger().error("This node must be run in a terminal (TTY).")
                return

            while rclpy.ok():
                key = None
                try:
                    key = get_key(0.1)
                except Exception as e:
                    self.get_logger().error(f"Key read error: {e}")
                    break

                if key is None:
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue

                if key in ('q', '\x03'):
                    break

                cmd = None
                if key in ('w', 'W', '\x1b[A'):
                    cmd = 'forward'
                elif key in ('s', 'S', '\x1b[B'):
                    cmd = 'backward'
                elif key == ' ':
                    cmd = 'stop'

                if cmd:
                    self.publish_cmd(cmd)

                rclpy.spin_once(self, timeout_sec=0.0)

        finally:
            # Always try to stop the robot and restore terminal settings
            try:
                self.publish_cmd('stop')
            except Exception:
                pass
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._orig_settings)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SnakeTeleop()
        node.loop()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
