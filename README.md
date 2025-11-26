# 🐢 Smart Turtle Controller (ROS 2 + Gemini AI)

This project implements a **Natural Language Interface** for the ROS 2 `turtlesim` robot. It uses the **Google Gemini API** to translate English commands (e.g., "Draw a pentagon") into executable movement trajectories.

## 🚀 Features

  * **AI-Powered Planning:** Uses Large Language Models (LLM) to understand geometric intent.
  * **JSON Protocol:** Enforces a strict structured output from the AI to ensure safety.
  * **ROS 2 Integration:** A custom Python node that executes open-loop velocity commands.
  * **Drift Mitigation:** Includes high-frequency timer logic (100Hz) to capture micro-movements.

## 🛠️ Prerequisites

  * **ROS 2** (Humble, Foxy, or Jazzy)
  * **Python 3.10+**
  * **Turtlesim package:** `sudo apt install ros-humble-turtlesim`
  * **Google GenAI SDK:** `pip install google-genai`

## 📦 Installation

1.  **Clone the repository** into your ROS 2 workspace `src` folder:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/YOUR_USERNAME/my_turtle_controller.git
    ```

2.  **Install Python dependencies:**

    ```bash
    pip install google-genai
    ```

3.  **Build the package:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_turtle_controller
    source install/setup.bash
    ```

## 🔑 Configuration

You need a **Google Gemini API Key** to run this node.

1.  Get a key from [Google AI Studio](https://aistudio.google.com/).
2.  **Security Best Practice:** Do not hardcode the key. Export it in your terminal before running:
    ```bash
    export GOOGLE_API_KEY="your_actual_api_key_here"
    ```
    *(Note: You may need to update the `smart_turtle.py` code to read `os.environ.get("GOOGLE_API_KEY")` if you haven't already).*

## 🐢 Usage

**Step 1: Launch the Simulation**
Open a terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

**Step 2: Run the AI Controller**
Open a second terminal, source your workspace, and run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_turtle_controller smart_turtle
```

**Step 3: Command the Robot**
The terminal will prompt you:

```text
Enter your command (e.g., 'Draw a square'):
```

Type something like: *"Draw a star pattern"* or *"Move in a zigzag."*

## 🧠 System Architecture

1.  **User Input:** Captures natural language string.
2.  **Prompt Engineering:** Wraps input with a System Prompt defining physical constraints (11x11m world) and JSON schema.
3.  **Gemini API:** Returns a list of primitives: `[{"command": "move", "value": 2.0}, ...]`.
4.  **Executor:** The ROS node parses the JSON and uses a 100Hz timer to publish `cmd_vel` messages based on calculated duration ($t = d/v$).

## ⚠️ Known Issues

  * **Drift:** Since this uses Open-Loop control (no odometry feedback), complex shapes may not close perfectly due to friction and timing variance.
  * **API Latency:** There is a slight delay while the AI generates the plan.

