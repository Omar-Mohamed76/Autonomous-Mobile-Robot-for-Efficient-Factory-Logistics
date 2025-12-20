#!/usr/bin/env python3

#if there is too much access on this node convert it to server better


import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import UInt8MultiArray

class BMSSENSOR(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("BMS_sensor") #call the init from the Node class 
        #uart serial don't forget to enable and select the serial
        self.ser_ = serial.Serial('/dev/serial0',9600,timeout=1.0) 
        self.BMS_pub_ = self.create_publisher(UInt8MultiArray,"BMS_data",10)
        self.command_sub_ = self.create_subscription(UInt8MultiArray,"BMS_command",self.callback_BMS_command_sub,10)
        self.get_logger().info("the_BMS_IS_Running...")

    def callback_BMS_command_sub(self, command :UInt8MultiArray):
        self.ser_.write(command.data)
        time.sleep(0.1)
        response = self.ser_.read(self.ser_.in_waiting)
        msg =UInt8MultiArray()
        msg.data= list(response)
        self.BMS_pub_.publish(msg)



def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=BMSSENSOR()   #create a object from the class
    rclpy.spin(node)        #make the node spin
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()