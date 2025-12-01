#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import json
import os
from google import genai


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

        # Current pose
        self.current_pose = None

        # Turtlesim boundaries (11x11 grid, 0.5 margin)
        self.min_x = 0.5
        self.max_x = 10.5
        self.min_y = 0.5
        self.max_y = 10.5

        # Mission state
        self.mission_plan = []
        self.current_step = 0
        self.cmd_start_time = 0.0
        self.busy = False
        self.executing = False

        # Pose-based control (for drift correction)
        self.start_pose = None
        self.target_distance = 0.0
        self.target_angle = 0.0

        # Wait for pose data
        self.get_logger().info("Waiting for turtle pose...")
        while self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Turtle ready at position ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})")

        # Start command loop
        self.command_loop()

    def pose_callback(self, msg):
        """Update current turtle position"""
        self.current_pose = msg

    def reset_turtle(self):
        """Reset turtle to center position using turtlesim reset service"""
        self.get_logger().info("Resetting turtlesim...")

        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Reset service not available")
            return

        request = Empty.Request()

        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            self.get_logger().info("Turtlesim reset complete!")
            # Wait a moment for pose to update
            rclpy.spin_once(self, timeout_sec=0.2)
        else:
            self.get_logger().error("Failed to reset turtlesim")

    def command_loop(self):
        """Main loop for taking user commands"""
        while rclpy.ok():
            print("\n" + "=" * 50)
            print("SMART TURTLE CONTROLLER")
            print("=" * 50)
            print("Enter '1' to reset turtle to center")
            print("Or describe what you want the turtle to draw:")
            print("Examples: 'draw a square', 'draw a triangle', 'move forward'")
            print("=" * 50)

            user_input = input("\nYour command: ").strip()

            if not user_input:
                continue

            # Check for reset command
            if user_input == "1":
                self.reset_turtle()
                continue

            # Check for exit
            if user_input.lower() in ['exit', 'quit', 'q']:
                self.get_logger().info("Exiting...")
                break

            # Get plan from Gemini
            self.get_logger().info(f"Processing: {user_input}")

            # Check for regular shapes - use optimized geometry
            if "circle" in user_input.lower():
                self.mission_plan = self.generate_perfect_circle(user_input)
            elif self.is_regular_polygon_request(user_input):
                self.mission_plan = self.generate_regular_polygon(user_input)
            else:
                self.mission_plan = self.ask_gemini_for_plan(user_input)

            if not self.mission_plan:
                self.get_logger().warn("No valid plan generated. Try again.")
                continue

            # Validate plan stays within boundaries
            if not self.validate_boundaries():
                self.get_logger().warn("Command would go out of bounds! Try a smaller shape or reset turtle.")
                continue

            # Execute the plan
            self.current_step = 0
            self.executing = True

            # Wait for execution to complete
            while self.executing and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info("Command completed!")

    def is_regular_polygon_request(self, user_request):
        """Detect if user wants a regular polygon"""
        request_lower = user_request.lower()

        # List of regular shapes
        regular_shapes = [
            'triangle', 'square', 'pentagon', 'hexagon',
            'heptagon', 'octagon', 'nonagon', 'decagon',
            'rectangle'  # We'll make it a square
        ]

        return any(shape in request_lower for shape in regular_shapes)

    def generate_regular_polygon(self, user_request):
        """Generate a perfect regular polygon based on user request"""
        import re

        request_lower = user_request.lower()

        # Map shape names to number of sides
        shape_map = {
            'triangle': 3,
            'square': 4,
            'rectangle': 4,  # Treat as square
            'pentagon': 5,
            'hexagon': 6,
            'heptagon': 7,
            'octagon': 8,
            'nonagon': 9,
            'decagon': 10
        }

        # Detect which shape
        sides = None
        shape_name = None
        for shape, num_sides in shape_map.items():
            if shape in request_lower:
                sides = num_sides
                shape_name = shape
                break

        if sides is None:
            self.get_logger().warn("Could not detect polygon type, defaulting to square")
            sides = 4
            shape_name = "square"

        # Extract side length if specified (e.g., "square with side 2")
        side_length = 2.0  # Default 2 meters
        size_match = re.search(r'side\s+(\d+\.?\d*)', request_lower)
        if size_match:
            side_length = float(size_match.group(1))
            self.get_logger().info(f"Using custom side length: {side_length}m")

        # Check if shape fits in workspace
        if not self.current_pose:
            self.get_logger().warn("No pose data, using default size")
        else:
            # Approximate bounding box (conservative estimate)
            max_dimension = side_length * 1.5  # Account for diagonal
            center_x = self.current_pose.x
            center_y = self.current_pose.y

            if (center_x - max_dimension / 2 < self.min_x or
                    center_x + max_dimension / 2 > self.max_x or
                    center_y - max_dimension / 2 < self.min_y or
                    center_y + max_dimension / 2 > self.max_y):
                old_length = side_length
                side_length = min(side_length, 2.0)  # Cap at 2m
                self.get_logger().warn(
                    f"Shape too large! Auto-adjusted from {old_length}m to {side_length}m"
                )

        # REGULAR POLYGON MATH:
        # - For a polygon with n sides
        # - Interior angle = (n-2) × 180° / n
        # - Exterior angle (turn angle) = 360° / n
        # - Move forward side_length, turn exterior_angle, repeat n times

        exterior_angle = 360.0 / sides

        self.get_logger().info(
            f"Generating perfect {shape_name}: {sides} sides, "
            f"side length={side_length:.2f}m, {exterior_angle:.1f}° turns"
        )

        # Build the command sequence
        plan = []
        for i in range(sides):
            plan.append({"command": "move", "value": side_length})
            plan.append({"command": "turn", "value": exterior_angle})

        self.get_logger().info(
            f"{shape_name.capitalize()} plan: {len(plan)} commands "
            f"({sides} sides, {exterior_angle * sides:.0f}° total rotation)"
        )
        return plan

    def generate_perfect_circle(self, user_request):
        """Generate a mathematically perfect circle"""
        # Default parameters
        radius = 1.0  # 1 meter radius
        segments = 36  # 36 segments = 10° per segment

        # Try to extract radius from user input (e.g., "draw a circle with radius 2")
        import re
        radius_match = re.search(r'radius\s+(\d+\.?\d*)', user_request.lower())
        if radius_match:
            radius = float(radius_match.group(1))
            self.get_logger().info(f"Using custom radius: {radius}m")

        # Check if circle fits in workspace
        if not self.current_pose:
            self.get_logger().warn("No pose data, using default radius")
        else:
            # Calculate if circle fits
            center_x = self.current_pose.x
            center_y = self.current_pose.y

            # Check all 4 extreme points would be in bounds
            if (center_x - radius < self.min_x or center_x + radius > self.max_x or
                    center_y - radius < self.min_y or center_y + radius > self.max_y):
                # Auto-adjust radius to fit
                max_radius_x = min(center_x - self.min_x, self.max_x - center_x)
                max_radius_y = min(center_y - self.min_y, self.max_y - center_y)
                old_radius = radius
                radius = min(max_radius_x, max_radius_y, 2.0)  # Cap at 2m for safety
                self.get_logger().warn(
                    f"Circle too large! Auto-adjusted radius from {old_radius}m to {radius}m"
                )

        # Circle math:
        # - Total rotation needed: 360 degrees
        # - Each segment: turn a bit, move forward to trace the arc
        # - Arc length for each segment = (angle_in_radians) × radius

        angle_per_segment = 360.0 / segments  # degrees
        angle_radians = angle_per_segment * 3.14159265359 / 180.0
        arc_length = angle_radians * radius

        self.get_logger().info(
            f"Generating perfect circle: {segments} segments, "
            f"radius={radius:.2f}m, {angle_per_segment:.1f}° per turn, "
            f"{arc_length:.4f}m per move"
        )

        # Build the command sequence
        plan = []
        for i in range(segments):
            plan.append({"command": "turn", "value": angle_per_segment})
            plan.append({"command": "move", "value": arc_length})

        self.get_logger().info(f"Circle plan: {len(plan)} total commands (360° total rotation)")
        return plan

    def ask_gemini_for_plan(self, user_request):
        """Ask Gemini to generate movement plan"""
        api_key = os.getenv("API_KEY")
        if not api_key:
            self.get_logger().error("API_KEY environment variable not set!")
            return []

        client = genai.Client(api_key=api_key)

        system_instruction = """
You are a Robot Controller for a turtle in an 11x11 meter workspace. 
You output ONLY raw JSON. No markdown, no explanations, no extra text.

Valid commands:
1. "move": value in meters (float). Positive=forward, Negative=backward.
2. "turn": value in degrees (float). Positive=Left (counterclockwise), Negative=Right (clockwise).

IMPORTANT RULES:
- Keep ALL movements small (maximum 2-3 meters)
- DO NOT generate regular polygons (triangle, square, pentagon, hexagon, etc.) - they are handled separately
- DO NOT generate circles - they are handled separately
- Only generate plans for: custom paths, spirals, stars, letters, numbers, or complex patterns
- Be creative but keep movements within bounds

Example star pattern:
[
    {"command": "move", "value": 2.0},
    {"command": "turn", "value": 144.0},
    {"command": "move", "value": 2.0},
    {"command": "turn", "value": 144.0},
    {"command": "move", "value": 2.0},
    {"command": "turn", "value": 144.0},
    {"command": "move", "value": 2.0},
    {"command": "turn", "value": 144.0},
    {"command": "move", "value": 2.0}
]
"""

        full_prompt = f"{system_instruction}\n\nUser Request: {user_request}"

        try:
            response = client.models.generate_content(
                model="gemini-2.0-flash",
                contents=full_prompt
            )

            raw_text = response.text
            self.get_logger().info(f"Gemini response received")

            # Clean and parse JSON
            cleaned_text = self.clean_json_response(raw_text)
            plan = json.loads(cleaned_text)

            self.get_logger().info(f"Plan generated with {len(plan)} steps")
            return plan

        except Exception as e:
            self.get_logger().error(f"Gemini failed: {e}")
            return []

    def clean_json_response(self, text):
        """Remove markdown formatting from JSON response"""
        text = text.strip()
        if text.startswith("```json"):
            text = text[7:]
        if text.startswith("```"):
            text = text[3:]
        if text.endswith("```"):
            text = text[:-3]
        return text.strip()

    def validate_boundaries(self):
        """Check if planned movements stay within turtlesim boundaries"""
        if not self.current_pose:
            return True

        # Simulate the path
        sim_x = self.current_pose.x
        sim_y = self.current_pose.y
        sim_theta = self.current_pose.theta

        for step in self.mission_plan:
            if step['command'] == 'move':
                distance = step['value']
                sim_x += distance * cos(sim_theta)
                sim_y += distance * sin(sim_theta)

                if not (self.min_x <= sim_x <= self.max_x and
                        self.min_y <= sim_y <= self.max_y):
                    return False

            elif step['command'] == 'turn':
                angle_rad = step['value'] * 3.14159265359 / 180.0
                sim_theta += angle_rad

        return True

    def timer_callback(self):
        """Execute current command step using pose feedback"""
        if not self.executing or not self.current_pose:
            return

        msg = Twist()

        # Check if mission complete
        if self.current_step >= len(self.mission_plan):
            self.publisher_.publish(msg)
            self.executing = False
            return

        instruction = self.mission_plan[self.current_step]

        if not self.busy:
            # Starting a new command - record start pose
            self.busy = True
            self.start_pose = Pose()
            self.start_pose.x = self.current_pose.x
            self.start_pose.y = self.current_pose.y
            self.start_pose.theta = self.current_pose.theta

            if instruction['command'] == 'move':
                self.target_distance = abs(instruction['value'])
            elif instruction['command'] == 'turn':
                self.target_angle = abs(instruction['value'] * 3.14159265359 / 180.0)

        # Execute movement commands with pose feedback
        if instruction['command'] == "move":
            # Calculate actual distance traveled
            dx = self.current_pose.x - self.start_pose.x
            dy = self.current_pose.y - self.start_pose.y
            distance_traveled = (dx * dx + dy * dy) ** 0.5

            remaining = self.target_distance - distance_traveled

            if remaining > 0.01:  # 1cm tolerance
                # Proportional control: slow down as we approach target
                base_speed = 1.2
                min_speed = 0.3

                # Speed proportional to remaining distance
                speed = max(min_speed, min(base_speed, base_speed * remaining / 0.5))

                msg.linear.x = speed if instruction['value'] > 0 else -speed
            else:
                # Ensure we stop completely
                msg.linear.x = 0.0
                self.finish_step()

        elif instruction['command'] == "turn":
            # Calculate actual angle turned (relative to start)
            angle_diff = self.current_pose.theta - self.start_pose.theta

            # Normalize angle to [-pi, pi]
            while angle_diff > 3.14159265359:
                angle_diff -= 2 * 3.14159265359
            while angle_diff < -3.14159265359:
                angle_diff += 2 * 3.14159265359

            # Determine if we should turn left (positive) or right (negative)
            target_signed = instruction['value'] * 3.14159265359 / 180.0

            # Check if we've reached the target angle
            error = target_signed - angle_diff

            if abs(error) > 0.015:  # ~0.86 degree tolerance
                # Proportional control: slow down as we approach target
                base_speed = 1.2
                min_speed = 0.3

                # Speed proportional to error (faster when far, slower when close)
                speed = max(min_speed, min(base_speed, base_speed * abs(error) / 0.5))

                # Turn in the direction of the error
                msg.angular.z = speed if error > 0 else -speed
            else:
                # Ensure we stop completely
                msg.angular.z = 0.0
                self.finish_step()

        self.publisher_.publish(msg)

    def finish_step(self):
        """Complete current step and move to next"""
        self.busy = False
        self.current_step += 1


def sin(angle):
    """Simple sine approximation"""
    import math
    return math.sin(angle)


def cos(angle):
    """Simple cosine approximation"""
    import math
    return math.cos(angle)


def main(args=None):
    rclpy.init(args=args)
    node = SmartTurtle()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()