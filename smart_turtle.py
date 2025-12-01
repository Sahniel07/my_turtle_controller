#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import os
from google import genai  # Import the library


class SmartTurtle(Node):
    def __init__(self):
        super().__init__('smart_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # --- ASK GEMINI FOR THE PLAN ---
        # "Draw a triangle" or "Draw a house"
        user_request = "Draw a square pattern"
        self.get_logger().info(f"Asking Gemini: {user_request}...")

        self.mission_plan = self.ask_gemini_for_plan(user_request)

        # STATE VARIABLES
        self.current_step = 0
        self.cmd_start_time = 0.0
        self.busy = False

    def ask_gemini_for_plan(self, user_request):
        client = genai.Client(api_key="AIzaSyBeLfjiLCLVejTA6QnYC5vS-tMQjkjkyZI")

        #The System Prompt(TheRules)
        system_instruction = """
                You are a Robot Controller. You output ONLY raw JSON. No markdown, no explanations.
                Valid commands:
                1. "move": value in meters (float).
                2. "turn": value in degrees (float). Positive=Left, Negative=Right.

                Example Output:
                [
                    {"command": "move", "value": 1.0},
                    {"command": "turn", "value": 90.0}
                ]
                """

        full_prompt = f"{system_instruction}\n\nUser Request: {user_request}"

        try:
            response = client.models.generate_content(
                model="gemini-2.0-flash",
                contents=full_prompt
            )

            raw_text = response.text
            self.get_logger().info(f"Gemini replied: {raw_text}")


            cleaned_text = self.clean_json_response(raw_text)
            plan = json.loads(cleaned_text)
            return plan


        except Exception as e:
            self.get_logger().error(f"Gemini failed: {e}")
            return []

    def clean_json_response(self, text):
        # Remove markdown code blocks if present
        text = text.strip()
        if text.startswith("```json"):
            text = text[7:]
        if text.startswith("```"):
            text = text[3:]
        if text.endswith("```"):
            text = text[:-3]
        return text.strip()

    def timer_callback(self):
        msg = Twist()

        # If mission is empty or done, stop.
        if not self.mission_plan or self.current_step >= len(self.mission_plan):
            self.publisher_.publish(msg)
            return

        instruction = self.mission_plan[self.current_step]
        current_time = self.get_clock().now().nanoseconds / 1e9

        if not self.busy:
            self.busy = True
            self.cmd_start_time = current_time
            self.get_logger().info(f"Executing: {instruction}")
            return

        elapsed = current_time - self.cmd_start_time

        # LOGIC ENGINE
        if instruction['command'] == "move":
            speed = 1.0
            duration = abs(instruction['value']) / speed
            if elapsed < duration:
                msg.linear.x = speed if instruction['value'] > 0 else -speed
            else:
                self.finish_step()

        elif instruction['command'] == "turn":
            speed = 1.0
            # Convert Degrees to Radians for duration calc
            target_radians = abs(instruction['value'] * (3.14159 / 180.0))
            duration = target_radians / speed

            if elapsed < duration:
                msg.angular.z = speed if instruction['value'] > 0 else -speed
            else:
                self.finish_step()

        self.publisher_.publish(msg)

    def finish_step(self):
        self.busy = False
        self.current_step += 1

def main(args=None):
    rclpy.init(args=args)
    node = SmartTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()