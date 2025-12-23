#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import time

try:
    from hx711_multi import HX711
except ImportError:
    print("Warning: hx711_multi not installed. Install with: pip install hx711-multi")
    HX711 = None


class LoadCellSensor(Node):
    def __init__(self):
        super().__init__("load_cell_sensor")
        
        # Declare parameters
        self.declare_parameter("dout_pin", 5)  # GPIO5 for data out
        self.declare_parameter("pd_sck_pin", 6)  # GPIO6 for clock
        self.declare_parameter("timer_hz", 0.1)  # 10Hz reading rate
        self.declare_parameter("reference_unit", 1.0)  # Calibration factor
        self.declare_parameter("offset", 0.0)  # Tare offset
        
        # Get parameters
        self.dout_pin_ = self.get_parameter("dout_pin").value
        self.pd_sck_pin_ = self.get_parameter("pd_sck_pin").value
        self.timer_hz_ = self.get_parameter("timer_hz").value
        self.reference_unit_ = self.get_parameter("reference_unit").value
        self.offset_ = self.get_parameter("offset").value
        
        # Initialize HX711
        if HX711 is None:
            self.get_logger().error("HX711 library not available!")
            return
            
        try:
            self.hx711_ = HX711(
                dout_pin=self.dout_pin_,
                pd_sck_pin=self.pd_sck_pin_,
                select_channel='A'  # Channel A for 128 gain
            )
            
            # Set reference unit (calibration factor)
            self.hx711_.set_reference_unit(self.reference_unit_)
            
            # Reset and tare
            self.hx711_.reset()
            time.sleep(0.1)
            
            self.get_logger().info("Load cell initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize HX711: {e}")
            return
        
        # Publisher for weight data
        self.weight_pub_ = self.create_publisher(Float32, "load_cell_weight", 10)
        
        # Service for taring (zeroing) the scale
        self.tare_service_ = self.create_service(
            Trigger,
            "tare_load_cell",
            self.callback_tare_service
        )
        
        # Timer for periodic readings
        self.timer_ = self.create_timer(self.timer_hz_, self.callback_read_weight)
        
        self.get_logger().info("Load Cell Sensor is running...")
        self.get_logger().info(f"Publishing to: load_cell_weight at {1/self.timer_hz_:.1f}Hz")

    def callback_read_weight(self):
        """Read weight from load cell and publish"""
        try:
            # Read weight (already calibrated and tared)
            weight = self.hx711_.get_weight_mean(readings=5)
            
            if weight is not None:
                # Apply offset if set
                weight = weight - self.offset_
                
                # Create and publish message
                msg = Float32()
                msg.data = float(weight)
                self.weight_pub_.publish(msg)
                
                self.get_logger().info(f"Weight: {weight:.2f} kg")
            else:
                self.get_logger().warn("Failed to read weight from load cell")
                
        except Exception as e:
            self.get_logger().error(f"Error reading load cell: {e}")

    def callback_tare_service(self, request, response):
        """Service callback to tare (zero) the load cell"""
        try:
            self.get_logger().info("Taring load cell...")
            self.hx711_.tare(times=15)
            
            response.success = True
            response.message = "Load cell tared successfully"
            self.get_logger().info("Load cell tared successfully")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to tare: {str(e)}"
            self.get_logger().error(f"Tare failed: {e}")
        
        return response

    def destroy_node(self):
        """Cleanup before shutdown"""
        try:
            self.hx711_.power_down()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LoadCellSensor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()