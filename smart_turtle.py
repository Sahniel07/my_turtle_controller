#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SmartTurtle(Node):
    def __init__(self):
        super().__init__('smart_turtle')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.3, self.timer_callback)

        self.mission_plan = [
            {"command": "move", "value": 2.0},   # Step 1: Move 2 meters
            {"command": "turn", "value": 90.0},  # Step 2: Turn 90 deg
            {"command": "move", "value": 1.0},   # Step 3: Move 1 meter
            {"command": "turn", "value": 180.0}  # Step 4: Turn around
        ]

        self.current_step_index = 0
        self.get_logger().info("Path plan loaded")

    def timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SmartTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()