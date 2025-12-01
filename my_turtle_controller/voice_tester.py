#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import json
import os
import sys

# New Import for Voice
import speech_recognition as sr
from google import genai

# Helper math functions
import math


class SmartTurtle(Node):
    def __init__(self):
        super().__init__('smart_turtle')

        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # Reset service client
        self.reset_client = self.create_client(Empty, '/reset')

        # Timer for command execution
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Voice Recognizer Setup
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Calibration for background noise
        self.get_logger().info("Calibrating microphone for background noise... (Silence please)")
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
        self.get_logger().info("Microphone Calibrated!")

        # Current pose
        self.current_pose = None

        # Turtlesim boundaries
        self.min_x = 0.5
        self.max_x = 10.5
        self.min_y = 0.5
        self.max_y = 10.5

        # Mission state
        self.mission_plan = []
        self.current_step = 0
        self.busy = False
        self.executing = False

        # Pose-based control
        self.start_pose = None
        self.target_distance = 0.0
        self.target_angle = 0.0

        # Wait for pose data
        self.get_logger().info("Waiting for turtle pose...")
        while self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Turtle ready at ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})")

        # Start loop
        self.command_loop()

    def pose_callback(self, msg):
        self.current_pose = msg

    def reset_turtle(self):
        self.get_logger().info("Resetting turtlesim...")
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Reset service not available")
            return
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is not None:
            self.get_logger().info("Reset complete!")
            rclpy.spin_once(self, timeout_sec=0.2)

    def get_voice_input(self):
        """Captures audio and converts to text"""
        print("\nListening... (Speak now)")

        try:
            with self.microphone as source:
                # Listen with a timeout so we don't hang forever
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)

            print("Processing audio...")
            text = self.recognizer.recognize_google(audio)
            print(f"--> You said: '{text}'")
            return text

        except sr.WaitTimeoutError:
            self.get_logger().warn("Listening timed out. No speech detected.")
            return None
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio.")
            return None
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results; {e}")
            return None

    def command_loop(self):
        while rclpy.ok():
            print("\n" + "=" * 50)
            print("SMART VOICE TURTLE")
            print("=" * 50)
            print("[ENTER]   : Press Enter to Speak")
            print("'t'       : Type a command manually")
            print("'q'       : Quit")
            print("=" * 50)

            mode = input("Selection: ").strip().lower()

            user_input = ""

            if mode == 'q' or mode == 'exit':
                break
            elif mode == 't':
                user_input = input("Type command: ").strip()
            else:
                # Default to voice on Enter or any other key
                voice_text = self.get_voice_input()
                if voice_text:
                    user_input = voice_text
                else:
                    continue

            if not user_input:
                continue

            # Command Processing Logic
            if "reset" in user_input.lower() or user_input == "1":
                self.reset_turtle()
                continue

            self.get_logger().info(f"Processing: {user_input}")

            # 1. Check Perfect Circle
            if "circle" in user_input.lower():
                self.mission_plan = self.generate_perfect_circle(user_input)
            # 2. Check Regular Polygons
            elif self.is_regular_polygon_request(user_input):
                self.mission_plan = self.generate_regular_polygon(user_input)
            # 3. Ask Gemini for complex stuff
            else:
                self.mission_plan = self.ask_gemini_for_plan(user_input)

            if not self.mission_plan:
                continue

            if not self.validate_boundaries():
                self.get_logger().warn("Path out of bounds.")
                continue

            # Execute
            self.current_step = 0
            self.executing = True
            while self.executing and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info("Done!")

    # --- GEOMETRY GENERATORS ---

    def is_regular_polygon_request(self, user_request):
        request_lower = user_request.lower()
        regular_shapes = ['triangle', 'square', 'pentagon', 'hexagon',
                          'heptagon', 'octagon', 'rectangle']
        return any(shape in request_lower for shape in regular_shapes)

    def generate_regular_polygon(self, user_request):
        import re
        request_lower = user_request.lower()
        shape_map = {'triangle': 3, 'square': 4, 'rectangle': 4, 'pentagon': 5,
                     'hexagon': 6, 'heptagon': 7, 'octagon': 8}

        sides = 4  # Default
        for shape, num in shape_map.items():
            if shape in request_lower:
                sides = num
                break

        side_length = 2.0
        # Look for numbers in speech (e.g. "size two", "side length 3")
        # Converting words to numbers is hard, relying on digits or simple parsing
        # Simple regex for digits:
        size_match = re.search(r'(\d+\.?\d*)', request_lower)
        if size_match:
            try:
                # Filter out '1' if it was part of "turtle1" or similar noise
                val = float(size_match.group(1))
                if 0.5 < val < 5.0:
                    side_length = val
            except:
                pass

        exterior_angle = 360.0 / sides
        plan = []
        for i in range(sides):
            plan.append({"command": "move", "value": side_length})
            plan.append({"command": "turn", "value": exterior_angle})
        return plan

    def generate_perfect_circle(self, user_request):
        radius = 1.0
        segments = 36
        angle_per_segment = 360.0 / segments
        angle_radians = math.radians(angle_per_segment)
        arc_length = angle_radians * radius

        plan = []
        for i in range(segments):
            plan.append({"command": "turn", "value": angle_per_segment})
            plan.append({"command": "move", "value": arc_length})
        return plan

    # --- AI & LOGIC ---

    def ask_gemini_for_plan(self, user_request):
        api_key = os.getenv("API_KEY")
        if not api_key:
            self.get_logger().error("API_KEY env var missing")
            return []

        client = genai.Client(api_key=api_key)

        # Added "Voice input" context to system instruction
        system_instruction = """
You are a Robot Controller. The user is speaking via voice commands, so the input might be informal.
Output ONLY raw JSON.
Valid commands: "move" (meters), "turn" (degrees).
Keep moves small (max 3m).
Example: [{"command": "move", "value": 1.0}]
"""
        try:
            response = client.models.generate_content(
                model="gemini-2.0-flash",
                contents=f"{system_instruction}\n\nUser Voice Request: {user_request}"
            )
            text = self.clean_json_response(response.text)
            return json.loads(text)
        except Exception as e:
            self.get_logger().error(f"AI Error: {e}")
            return []

    def clean_json_response(self, text):
        text = text.strip()
        if text.startswith("```json"): text = text[7:]
        if text.startswith("```"): text = text[3:]
        if text.endswith("```"): text = text[:-3]
        return text.strip()

    def validate_boundaries(self):
        if not self.current_pose: return True
        sim_x, sim_y, sim_theta = self.current_pose.x, self.current_pose.y, self.current_pose.theta

        for step in self.mission_plan:
            if step['command'] == 'move':
                dist = step['value']
                sim_x += dist * math.cos(sim_theta)
                sim_y += dist * math.sin(sim_theta)
                if not (self.min_x <= sim_x <= self.max_x and self.min_y <= sim_y <= self.max_y):
                    return False
            elif step['command'] == 'turn':
                sim_theta += math.radians(step['value'])
        return True

    def timer_callback(self):
        if not self.executing or not self.current_pose: return

        msg = Twist()
        if self.current_step >= len(self.mission_plan):
            self.publisher_.publish(msg)
            self.executing = False
            return

        instruction = self.mission_plan[self.current_step]

        if not self.busy:
            self.busy = True
            self.start_pose = Pose()
            self.start_pose.x, self.start_pose.y, self.start_pose.theta = \
                self.current_pose.x, self.current_pose.y, self.current_pose.theta

            if instruction['command'] == 'move':
                self.target_distance = abs(instruction['value'])
            elif instruction['command'] == 'turn':
                self.target_angle = abs(math.radians(instruction['value']))

        if instruction['command'] == "move":
            dx = self.current_pose.x - self.start_pose.x
            dy = self.current_pose.y - self.start_pose.y
            dist_traveled = math.sqrt(dx ** 2 + dy ** 2)
            remaining = self.target_distance - dist_traveled

            if remaining > 0.01:
                speed = max(0.2, min(1.2, 1.2 * remaining / 0.5))
                msg.linear.x = speed if instruction['value'] > 0 else -speed
            else:
                self.finish_step()

        elif instruction['command'] == "turn":
            angle_diff = self.current_pose.theta - self.start_pose.theta
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi

            target_rad = math.radians(instruction['value'])
            error = target_rad - angle_diff

            if abs(error) > 0.015:
                speed = max(0.3, min(1.2, 1.2 * abs(error) / 0.5))
                msg.angular.z = speed if error > 0 else -speed
            else:
                self.finish_step()

        self.publisher_.publish(msg)

    def finish_step(self):
        self.busy = False
        self.current_step += 1


def main(args=None):
    rclpy.init(args=args)
    node = SmartTurtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()