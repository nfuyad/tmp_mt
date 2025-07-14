#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import serial

class CmdToSerial(Node):
    def __init__(self):
        super().__init__("cmd_to_serial")

        #Setting Serial

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)

        serial_port= self.get_parameter("serial_port").value
        baud_rate= self.get_parameter("baud_rate").value

        try:
            self.serial_communication= serial.Serial(serial_port, baud_rate, timeout= 1)
            self.get_logger().info(f"Connected to serial port: {serial_port} at {baud_rate}")

        except serial.SerialException as e:
            self.get_logger().error(f"Process failed {e}")
            raise SystemExit

        self.last_twist= Twist()

        self.sub_= self.create_subscription(Twist, "/cmd_vel", self.callback, 10)
        # self.arm_subscription_= self.create_subscription(Float64MultiArray, "/arm_array_value", self.arm_joy_callback, 1)

    def callback(self, msg):
        self.last_twist= msg
        command= f"{self.last_twist.linear.x},{self.last_twist.angular.z}\n"

        try:

            self.serial_communication.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent to serial {command}")
        
        except serial.SerialException as e:
            self.get_logger().error(f"Process failed {e}")
            raise SystemExit
        
    def arm_joy_callback(self, msg):
        
        command= ",".join(map(str, msg.data)) + "\n"

        try:

            self.serial_communication.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent to serial {command.strip()}")
        
        except serial.SerialException as e:
            self.get_logger().error(f"Process failed {e}")
            raise SystemExit
        
    def destroy_node(self):
        if self.serial_communication.is_open:
            self.serial_communication.close()
            self.get_logger().info("Seial closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node= CmdToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()