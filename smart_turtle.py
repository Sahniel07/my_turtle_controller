#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class SmartTurtle(Node):
    def __init__(self):
        super().__init__('smart_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # THE PLAN
        self.mission_plan = [
            {"command": "move", "value": 2.0},
            {"command": "turn", "value": 90.0},
            {"command": "move", "value": 1.0},
            {"command": "turn", "value": 180.0}
        ]

        self.current_step = 0
        self.cmd_start_time = 0.0
        self.busy = False

    def timer_callback(self):
        msg = Twist()

        # 1. Check if mission is done
        if self.current_step >= len(self.mission_plan):
            self.publisher_.publish(msg)  # Stop
            return

        instruction = self.mission_plan[self.current_step]
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 2. START NEW COMMAND
        if not self.busy:
            self.busy = True
            self.cmd_start_time = current_time
            self.get_logger().info(f"Starting: {instruction}")
            return  # Wait for next tick to start moving

        # 3. EXECUTE COMMAND
        elapsed = current_time - self.cmd_start_time

        if instruction['command'] == "move":
            speed = 1.0
            duration = abs(instruction['value']) / speed

            if elapsed < duration:
                msg.linear.x = speed if instruction['value'] > 0 else -speed
            else:
                self.finish_step()

        elif instruction['command'] == "turn":
            speed = 1.0  # 1 radian per second
            target_radians = abs(math.radians(instruction['value']))
            duration = target_radians / speed

            if elapsed < duration:
                msg.angular.z = speed if instruction['value'] > 0 else -speed
            else:
                self.finish_step()

        self.publisher_.publish(msg)

    def finish_step(self):
        self.busy = False
        self.current_step += 1
        self.get_logger().info("Step Done.")


def main(args=None):
    rclpy.init(args=args)
    node = SmartTurtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()