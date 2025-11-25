#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.counter_ = 0

    def timer_callback(self):
        msg = Twist()
        if self.counter_ < 4:
            msg.linear.x = 1.0
            msg.angular.z = 0.0 #so no turning here
            self.get_logger().info("Straight")

        elif self.counter_ < 6:
            msg.linear.x = 0.0
            msg.angular.z = 1.57
            self.get_logger().info("Turns")

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.counter_ = -1
        self.publisher_.publish(msg)
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()