#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class ArmController(Node):
    def __init__(self):
        super().__init__("arm_control")

        self.min_vel= 0
        self.max_vel= 300

        self.pub_= self.create_publisher(Float64MultiArray, "arm_array_value", 10)
        
        self.sub_= self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        

    def joy_callback(self, msg: Joy):
        msgarry= Float64MultiArray()
        msgarry.data= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if(msg.buttons[1]):
            msgarry.data[0]= msg.buttons[1]*self.max_vel
        if(msg.buttons[1]):
            msgarry.data[0]= msg.buttons[1]*self.max_vel
        if(msg.buttons[1]):
            msgarry.data[0]= msg.buttons[1]*self.max_vel
        if(msg.buttons[1]):
            msgarry.data[0]= msg.buttons[1]*self.max_vel
        if(msg.buttons[1]):
            msgarry.data[0]= msg.buttons[1]*self.max_vel
        if(msg.buttons[1]):
            msgarry.data[0]= msg.buttons[1]*self.max_vel

        self.pub_.publish(msgarry)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 

