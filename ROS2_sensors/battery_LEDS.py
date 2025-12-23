#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from gpiozero import LED

class BMSLedSubscriber(Node):
    def __init__(self):
        super().__init__("bms_led_subscriber")
        
        # --- LED Initialization ---
        # Using GPIO Zero for clean pin management
        self.led_one = LED(27)
        self.led_two = LED(22)
        self.led_three = LED(23)
        
        # --- Subscriber ---
        # Subscribes to the topic published by your BMS node
        self.subscription = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            10)
        
        self.get_logger().info("BMS LED Subscriber Node Started - Waiting for battery data...")

    def battery_callback(self, msg):
        # ROS 2 BatteryState percentage is usually 0.0 to 1.0
        percent = msg.percentage * 100
        
        self.get_logger().info(f"Received Battery Level: {percent}%")

        # --- LED Logic Hierarchy ---
        if percent >= 100:
            # 100% - All three LEDs ON
            self.led_one.on()
            self.led_two.on()
            self.led_three.on()
        elif percent >= 66:
            # 66% - two and three ON
            self.led_one.off()
            self.led_two.on()
            self.led_three.on()
        elif percent >= 33:
            # 33% - Only three ON
            self.led_one.off()
            self.led_two.off()
            self.led_three.on()
        else:
            # Below 33% - All LEDs OFF (or you could make three blink)
            self.led_one.off()
            self.led_two.off()
            self.led_three.off()

def main(args=None):
    rclpy.init(args=args)
    node = BMSLedSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: Turn off LEDs on shutdown
        node.led_one.off()
        node.led_two.off()
        node.led_three.off()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()