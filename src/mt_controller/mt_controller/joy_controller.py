#! /urs/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

class JoyStickControl(Node):
    def __init__(self):
        super().__init__("joy_controller")
        
        self.declare_parameter("Lin", 255.0)
        self.declare_parameter("Ang", 255.0)

        self.lin= self.get_parameter("Lin").value
        self.ang= self.get_parameter("Ang").value

        self.time= True

        self.lastvaluelin=0
        self.lastvalueang=0

        self.pub_= self.create_publisher(Twist, "/cmd_vel", 10)

        self.sub_= self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.timer_= self.create_timer(1, self.call_back)


    def call_back(self):
        self.time= True
        


    def joy_callback(self, msg: Joy):
        if(msg.axes[6]):
            self.lin-= 10.0
        elif(msg.axes[7]):
            self.ang+= 10.0
        if False:
            pass

        else:

            twist= Twist()

            value = msg.axes[1]
            rounded = math.floor(value * 10) / 10
            linearcap= rounded*self.lin

            if(abs(rounded-self.lastvaluelin)>0.1):
                if(-50.0<linearcap<50.0):
                    rounded= 0.0
                twist.linear.x= rounded*self.lin
                twist.angular.z= 0.0
                self.lastvaluelin=rounded
            
                self.pub_.publish(twist)

            #Angular


            value = msg.axes[3]
            rounded = math.floor(value * 10) / 10
            angularcap= rounded*self.ang

            if(abs(rounded-self.lastvalueang)>0.1):
                if(-50.0<angularcap<50.0):
                    rounded= 0.0
                twist.linear.x= 0.0
                twist.angular.z= rounded*self.ang
                self.lastvalueang=rounded
            
                self.pub_.publish(twist)
            self.time= False


def main(args=None):
    rclpy.init(args=args)
    node = JoyStickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


