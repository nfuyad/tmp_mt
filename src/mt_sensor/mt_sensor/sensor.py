import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import serial

class SimpleSensorRead(Node):
    def __init__(self):
        super().__init__('sensor_value')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', '9600')

        port= self.get_parameter('port').value
        baud= self.get_parameter('baud').value

        self.message= ""
        self.state= None
        
        self.comm_= serial.Serial(port, baud, timeout= 1)

        self.pub_= self.create_publisher(Float64MultiArray, '/mt_sensor', 10)
        self.pub_temp_= self.create_publisher(Float64MultiArray, '/temp_hum', 10)
        self.pub_oxy_= self.create_publisher(Float64MultiArray, '/oxygen_sensor', 10)
        self.pub_light_= self.create_publisher(Float64MultiArray, '/light_sensor', 10)
        self.pub_bmp_= self.create_publisher(Float64MultiArray, '/bmp_sensor', 10)
        self.color_ = self.create_publisher(Float64MultiArray,'/color', 10)

        self.timer= self.create_timer(1, self.timer_callback)

        #Sensors data
        self.s1= Float64MultiArray()
        self.temp_hum= Float64MultiArray()
        self.oxygen= Float64MultiArray()
        self.light= Float64MultiArray()
        self.bmp= Float64MultiArray()
        self.s6= Float64MultiArray()
        self.color_obj= Float64MultiArray()

        self.s1.data= [0.0]
        self.temp_hum.data= [0.0, 0.0] # Temperature, Humidity
        self.oxygen.data= [0.0] #oxygen
        self.light.data= [0.0] # Light Intensity
        self.bmp.data= [0.0, 0.0, 0.0] #Pressure, Temperature, Altimeter
        self.color_obj.data= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        


    def timer_callback(self):
        # print(f"Sensor 1:{self.s1.data}")
        # self.s1.data[0]+= 10
        # self.pub_.publish(self.s1)
        msg= self.readline()
        self.process_msg(msg)
        if (self.state== 7):
            self.pub_temp_.publish(self.temp_hum)
            self.pub_oxy_.publish(self.oxygen)
            self.pub_light_.publish(self.light)
            self.pub_bmp_.publish(self.bmp)

    def test_func(self):
        print("START")
        while True:
            self.pub_.publish(self.s1)

    def readline(self):
        msg= ""
        sensor_name= ""
        read= False

        while True:
            byte_msg= self.comm_.read().decode('utf-8', errors= 'ignore')

            if(byte_msg!= ")"):

                msg+= byte_msg
                # sensor_name+= byte_msg
            elif(byte_msg==")"):
                #Process and Publish
                self.message= msg
                # self.process_msg(msg)
                msg= ""
                # self.pub_.publish(self.s1)
                return self.message


    def process_msg(self, recieved_string):
        # lst= recieved_string.split("|")
        # lst1= lst[1::2]
        # sensor_values=[]
        # for i in lst1:
        #     sensor_values.append(float(i))

        """
        Processes the received string by splitting it and converting values to floats.
        Returns a list of floats.
        """
        lst = recieved_string.split("|")  # Split data by pipe
        lst1 = lst[1::2]  # Extract values after the first item
        sensor_values = []

        try:
            # Convert each value to a float
            for i in lst1:
                sensor_values.append(float(i))
        except ValueError:
            print("Error: Invalid data format received.")
            return []
        
        print(sensor_values)

        self.state= len(sensor_values)

        if(len(sensor_values)==7):

            self.temp_hum.data[0]= sensor_values[0]
            self.temp_hum.data[1]= sensor_values[1]

            #O2 sensor
            self.oxygen.data[0]= sensor_values[2]

            #light intensity
            self.light.data[0]= sensor_values[3]
            
            #BMP
            self.bmp.data[0]= sensor_values[4]
            self.bmp.data[1]= sensor_values[5]
            self.bmp.data[2]= sensor_values[6]
        

                



def main(args=None):

    rclpy.init(args=args)
    node= SimpleSensorRead()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

