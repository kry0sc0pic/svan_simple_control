# SVAN Simple Control

This package provides a simple control interface for the SVAN robot, allowing you to control movement, orientation, and operational modes using ROS messages or HTTP API.

## Setup Instructions

### Initial Setup

```bash
# Navigate to your ROS workspace source directory
cd ~/xMo/src

# Clone the main control repository
git clone https://github.com/kry0sc0pic/svan_simple_control.git

# Clone the messages repository (required for the message definitions)
git clone https://github.com/kry0sc0pic/svan_simple_control_msgs.git

# Build the packages
cd ~/xMo
catkin build svan_simple_control_msgs
source devel/setup.bash
catkin build svan_simple_control
source devel/setup.bash
```

### Setting up the HTTP Bridge

```bash
# Navigate to the svan_simple_control directory
cd ~/xMo/src/svan_simple_control

# Create the virtual environment
python3 -m venv bridge_venv

# Activate the virtual environment
source bridge_venv/bin/activate

# Install the required packages
pip install -r bridge/requirements.txt
```

## System Architecture

The SVAN control system has the following components:

1. **ROS Core (roscore)**: The ROS master that allows all nodes to communicate.

2. **Simulation or Hardware Interface**: Either:
   - Gazebo simulation for testing in a virtual environment
   - Hardware interface when working with the physical robot

3. **MCP (Master Control Program)**: Interfaces between the high-level commands and the robot.

4. **Simple Control Node**: Translates simplified commands into low-level control signals.

5. **HTTP Bridge**: An API gateway that allows controlling the robot via web requests instead of direct ROS messages.

## Running SVAN Simulation

Open two terminals, navigate to the `xMo` folder in both and run the following command on both:

```bash
source devel/setup.bash
```

### Terminal 1:

*This launches the Gazebo simulation and the keyboard control interface.*

```bash
roslaunch svan_bringup gazebo_sim.launch
```

### Terminal 2:

*This launches the motor controller node which converts keyboard control inputs into commands for the simulated SVAN M2*

```bash
python3 src/svan_control/src/mcp.py
```

## Running the Complete System with HTTP Bridge

You'll need 6 terminals to run the complete system:

### Terminal 1: ROS Core
```bash
# Start ROS Core
roscore
```

### Terminal 2: Gazebo Simulation (for simulation mode only)
```bash
# If using simulation, start Gazebo
# This depends on your specific launch file
# Example (may vary based on your setup):
roslaunch svan_gazebo svan.launch
```

### Terminal 3: MCP Interface
```bash
# Navigate to your workspace
cd ~/xMo

# Source the ROS environment
source devel/setup.bash

# Run the MCP program
rosrun svan_simple_control mcp.py
```

### Terminal 4: Simple Control Node
```bash
# Navigate to your workspace
cd ~/xMo

# Source the ROS environment
source devel/setup.bash

# For simulation:
rosrun svan_simple_control simulation.py

# OR for hardware:
# rosrun svan_simple_control hardware.py
```

### Terminal 5: HTTP Bridge
```bash
# Navigate to the svan_simple_control directory
cd ~/xMo/src/svan_simple_control

# Activate the virtual environment
source bridge_venv/bin/activate

# Source the ROS environment (IMPORTANT: do this AFTER activating the virtual environment)
source ~/xMo/devel/setup.bash

# Start the bridge
python3 bridge/bridge.py
```

### Terminal 6: Run Control Scripts
```bash
# For HTTP-based examples:
cd ~/xMo/src/svan_simple_control
source bridge_venv/bin/activate
python3 bridge/examples/pushup_http.py

# OR for direct ROS examples:
cd ~/xMo
source devel/setup.bash
python3 src/svan_simple_control/examples/pushup.py
```

## Using the HTTP Bridge

The HTTP bridge provides a REST API to control the SVAN robot using HTTP requests instead of direct ROS messages.

### API Endpoints

- `GET /` - Check if the bridge is running
- `GET /history` - View command history 
- `POST /mode` - Set operation mode
- `POST /movement` - Control linear movement
- `POST /roll` - Control roll angle
- `POST /pitch` - Control pitch angle
- `POST /yaw` - Control yaw direction
- `POST /height` - Control height

### Example Usage

```bash
# Set to TROT mode
curl -X POST http://127.0.0.1:8888/mode \
  -H "Content-Type: application/json" \
  -d '{"operation_mode": 4}'

# Move forward at half speed
curl -X POST http://127.0.0.1:8888/movement \
  -H "Content-Type: application/json" \
  -d '{"vel_x": 0.0, "vel_y": 0.5}'
```

The bridge comes with example scripts to demonstrate HTTP-based control:
- `bridge/example_client.py` - General example of sequential commands
- `bridge/examples/pushup_http.py` - HTTP-based pushup demonstration

For detailed setup and running instructions, see the [bridge/README.md](bridge/README.md) file.

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

### Direct ROS Examples
- `examples/pushup.py` - Demonstration of robot pushup mode using direct ROS messages
- `examples/rotate.py` - Demonstration of robot rotation control using direct ROS messages

### HTTP Bridge Examples
- `bridge/examples/pushup_http.py` - Same pushup demonstration but using HTTP requests

## Troubleshooting

### Common Issues and Solutions

#### "No module named 'svan_simple_control_msgs'"
This means the ROS environment isn't properly sourced in your virtual environment. Fix it by:
```bash
# Make sure you source the ROS workspace AFTER activating the virtual environment
source bridge_venv/bin/activate
source ~/xMo/devel/setup.bash
```

#### "Unable to register with master node"
This means ROS core isn't running. Start it in a separate terminal:
```bash
roscore
```

#### "Connection refused" when accessing the API
This means the bridge isn't running or is using a different port. Make sure:
1. The bridge is running (Terminal 5)
2. You're using the correct URL (http://127.0.0.1:8888)

#### Commands not reaching the robot
1. Check if all terminals are running correctly
2. Verify the bridge is not in mock mode (check the startup message)
3. Check the command history at `/history` to verify your commands are being received
4. For detailed logs, check the output in all terminal windows