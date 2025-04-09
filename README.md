# svan_simple_control

This package provides a simple control interface for the SVAN robot, allowing you to control movement, orientation, and operational modes using ROS messages or HTTP API.

## Setup Instructions

### Ubuntu Setup
```bash
# Install ROS dependencies (if not already installed)
sudo apt update
sudo apt install -y python3-catkin-tools python3-rosdep ros-noetic-catkin python3-venv

# Create and initialize workspace (if you don't have one already)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init

# Clone the repositories
cd ~/catkin_ws/src
git clone https://github.com/orionop/svan_simple_control.git
git clone https://github.com/kry0sc0pic/svan_simple_control_msgs.git


# For the messages package, contact repo maintainer for correct URL

# Install dependencies
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the packages
catkin build svan_simple_control_msgs
source devel/setup.bash
catkin build svan_simple_control
source devel/setup.bash

# Optional: Add to your .bashrc for convenience
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Understanding the System Architecture

The SVAN control system has the following components:

1. **ROS Core (roscore)**: The ROS master that allows all nodes to communicate.

2. **Simulation or Hardware Interface**: Either:
   - Gazebo simulation for testing in a virtual environment
   - Hardware interface when working with the physical robot

3. **MCP (Master Control Program)**: Interfaces between the high-level commands and the robot.

4. **Simple Control Node**: Translates simplified commands into low-level control signals.

5. **Bridge (Optional)**: An HTTP API gateway that allows controlling the robot via web requests instead of direct ROS messages.

## Running the Bridge

The bridge provides a REST API to control the robot using HTTP requests instead of directly using ROS messages.

```bash
# Create and activate a virtual environment
python3 -m venv bridge_venv
source bridge_venv/bin/activate

# Install required packages
pip install -r bridge/requirements.txt

# Start ROS Core in a separate terminal
roscore

# In a new terminal, run the bridge
source bridge_venv/bin/activate
python3 bridge/bridge.py
```

Once running, you can access the API documentation at: http://127.0.0.1:8888/docs

If you see "Unable to register with master node [http://localhost:11311]", it means roscore is not running. You need to:

1. Start roscore in a separate terminal: `roscore`
2. Leave that terminal running
3. Return to your bridge terminal and restart the bridge

## Running in Simulation Mode

To run the system in simulation mode, follow these steps in order:

```bash
# Terminal 1: Start roscore (must be first)
roscore

# Terminal 2: Start the Gazebo simulation
# This depends on your specific Gazebo launch file

# Terminal 3: Start the MCP interface
cd ~/xMo
source devel/setup.bash
# Replace with your actual MCP path - this may be different in your setup
rosrun svan_simple_control mcp.py

# Terminal 4: Run the simple control node
cd ~/xMo
source devel/setup.bash
rosrun svan_simple_control simulation.py

# Terminal 5: Run your control scripts or examples
cd ~/xMo
source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
```

## Running in Hardware Mode

To run on the physical SVAN robot:

```bash
# Complete all hardware startup steps first
# (joystick calibration, sleep calibration, moetus interface launch)

# Terminal 1: Start the MCP program
cd ~/dev/xMo
source devel/setup.bash
python3 mcp.py  # Path may vary

# Terminal 2: Run the hardware interface
cd ~/dev/xMo
source devel/setup.bash
rosrun svan_simple_control hardware.py

# Terminal 3: Run your control script
cd ~/dev/xMo
source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
```

## Message Definitions

SVAN is controlled using the `SvanCommand` message type with the following fields:

### `command_type` (`uint8`)
Specifies what aspect of the robot you want to control:

| Value | Aspect |
| ----- | ------ |
| 0 | OPERATION MODE (trot, push, up, etc.) |
| 1 | LINEAR MOVEMENT |
| 2 | ROLL ANGLE |
| 3 | PITCH ANGLE | 
| 4 | YAW VELOCITY |
| 5 | HEIGHT |

### `operation_mode` (`uint8`)
Used when `command_type` is `0` to set the robot's operation mode:

| Value | Mode |
| ----- | ---- |
| 1 | STOP |
| 2 | TWIRL |
| 3 | PUSHUP |
| 4 | TROT |
| 5 | SLEEP |

### `height` (`uint8`)
Used when `command_type` is `5` to set the robot's height:

| Value | Height |
| ----- | ------ |
| 1 | UP (MAX) |
| 2 | DOWN (MIN) |

### Movement Control
- `vel_x` (`float32`): Normalized velocity along X-axis (-1.0 to 1.0). Positive X is to the right.
- `vel_y` (`float32`): Normalized velocity along Y-axis (-1.0 to 1.0). Positive Y is to the front.

### Orientation Control
- `roll` (`float32`): Normalized roll angle (-1.0 = left, 0 = center, 1.0 = right)
- `pitch` (`float32`): Normalized pitch angle (1.0 = front, 0 = center, -1.0 = back)
- `yaw` (`uint8`): Yaw direction (0 = LEFT, 1 = RIGHT, 2 = NONE)

## Examples

The package includes several example scripts:

### Sine Wave Circle
Moves in a circular path while varying height.
```bash
python3 examples/sine_wave_circle.py
```

### Hand Gesture Movement Control
Move and stop the SVAN using hand gestures (requires MediaPipe).
```bash
python3 examples/hand_control.py
```

### Hand Gesture Height Control
Control the SVAN's height using gestures (requires MediaPipe).
```bash
python3 examples/height_control.py
```
