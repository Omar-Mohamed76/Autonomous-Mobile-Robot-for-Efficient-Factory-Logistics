#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from hx711_multi import HX711 

class LoadCellNode(Node):
    def __init__(self):
        super().__init__('load_cell_node')
        
        # Initialized on GPIO 5 (DT) and 6 (SCK) to avoid current hardware conflicts
        self.hx = HX711(dout_pin=5, sck_pin=6)
        
        # Set calibration factor (must be tuned with known weight)
        self.hx.set_scale(2280.0) 
        self.hx.tare() 

        self.publisher_ = self.create_publisher(Float32, 'payload_weight', 10)
        self.timer = self.create_timer(0.1, self.publish_weight)

    def publish_weight(self):
        try:
            weight = self.hx.get_units(1)
            msg = Float32()
            msg.data = float(weight)
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Load Cell Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LoadCellNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()