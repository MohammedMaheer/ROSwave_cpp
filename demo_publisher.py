#!/usr/bin/env python3
"""
Demo ROS2 publisher to test the dashboard with sample data.
Publishes to multiple topics with varying frequencies.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int32
from geometry_msgs.msg import Twist
import time
import math

class DemoPublisher(Node):
    def __init__(self):
        super().__init__('demo_publisher')
        
        # Create publishers for different topics
        self.string_pub = self.create_publisher(String, '/demo/message', 10)
        self.float_pub = self.create_publisher(Float64, '/demo/sensor', 10)
        self.int_pub = self.create_publisher(Int32, '/demo/counter', 10)
        self.velocity_pub = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        
        # Create timers for different frequencies
        self.create_timer(1.0, self.publish_string)      # 1 Hz
        self.create_timer(0.1, self.publish_float)        # 10 Hz
        self.create_timer(0.5, self.publish_int)          # 2 Hz
        self.create_timer(0.2, self.publish_velocity)     # 5 Hz
        
        self.counter = 0
        self.sensor_value = 0.0
        self.get_logger().info('Demo publisher started - publishing on /demo/* topics')
    
    def publish_string(self):
        msg = String()
        msg.data = f'Message #{self.counter}'
        self.string_pub.publish(msg)
        self.get_logger().info(f'Published string: {msg.data}')
        self.counter += 1
    
    def publish_float(self):
        msg = Float64()
        msg.data = 10.0 + 5.0 * math.sin(time.time())
        self.float_pub.publish(msg)
        self.get_logger().info(f'Published sensor value: {msg.data:.2f}')
    
    def publish_int(self):
        msg = Int32()
        msg.data = int(time.time()) % 1000
        self.int_pub.publish(msg)
        self.get_logger().info(f'Published counter: {msg.data}')
    
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 1.0 * math.cos(time.time())
        msg.linear.y = 1.0 * math.sin(time.time())
        msg.angular.z = 0.5 * math.sin(time.time() * 2)
        self.velocity_pub.publish(msg)
        self.get_logger().info(f'Published velocity: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    publisher = DemoPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Demo publisher stopped')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
