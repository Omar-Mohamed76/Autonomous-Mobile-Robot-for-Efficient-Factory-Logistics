#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import LED 

class BatteryLEDNode(Node):
    def __init__(self):
        super().__init__('battery_leds')
        
        self.led_red = LED(4)   
        self.led_yellow = LED(27) 
        self.led_green = LED(22)  

        self.subscription = self.create_subscription(
            Float32, 'battery_voltage', self.voltage_callback, 10)

    def voltage_callback(self, msg):
        v = msg.data
        if v < 21.0: # Red (Low)
            self.set_leds(r=True, y=False, g=False)
        elif 21.0 <= v < 24.0: # Yellow (Med)
            self.set_leds(r=False, y=True, g=False)
        else: # Green (Full)
            self.set_leds(r=False, y=False, g=True)

    def set_leds(self, r, y, g):
        self.led_red.value = r
        self.led_yellow.value = y
        self.led_green.value = g

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BatteryLEDNode())
    rclpy.shutdown()