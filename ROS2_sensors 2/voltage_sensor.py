#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class VoltageSensorNode(Node):
    def __init__(self):
        super().__init__('voltage_sensor_node')
        
        # Shared I2C Bus 1 (Pins 3 and 5)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c)
        self.chan = AnalogIn(self.ads, ADS.P0)
        
        # The module reduces input voltage by a factor of 5
        self.DIVIDER_RATIO = 5.0 
        
        self.voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.timer = self.create_timer(1.0, self.read_voltage)

    def read_voltage(self):
        # actual_v = divider_reading * 5
        actual_voltage = self.chan.voltage * self.DIVIDER_RATIO
        msg = Float32()
        msg.data = float(actual_voltage)
        self.voltage_pub.publish(msg)
        self.get_logger().info(f'Voltage: {actual_voltage:.2f}V')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VoltageSensorNode())
    rclpy.shutdown()