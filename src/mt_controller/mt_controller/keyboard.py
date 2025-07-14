#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class WASDTeleopNode(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_speed = 90.0
        self.angular_speed = 20.0
        self.move_bindings = {
            'w': (self.linear_speed, 0.0),
            's': (-self.linear_speed, 0.0),
            'a': (0.0, self.angular_speed),
            'd': (0.0, -self.angular_speed)
        }
        self.get_logger().info("WASD Teleop Node Initialized. Use 'W', 'A', 'S', 'D' to move the robot.")

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key in self.move_bindings:
                    twist = Twist()
                    twist.linear.x ,twist.angular.z = self.move_bindings[key]
                    self.publisher.publish(twist)
                elif key == '\x03':
                    break
                else:
                    twist = Twist()
                    self.publisher.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            self.restore_terminal()
            self.get_logger().info("Shutting down WASD Teleop Node.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()