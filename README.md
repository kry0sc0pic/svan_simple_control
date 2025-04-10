# SVAN Simple Control

This package provides a simple control interface for the SVAN robot, allowing you to control movement, orientation, and operational modes using ROS messages or HTTP API.

## System Architecture

The SVAN control system has the following components:

1. **ROS Core (roscore)**: The ROS master that allows all nodes to communicate.

2. **Simulation or Hardware Interface**: Either:
   - Gazebo simulation for testing in a virtual environment
   - Hardware interface when working with the physical robot

3. **MCP (Master Control Program)**: Interfaces between the high-level commands and the robot.

4. **Simple Control Node**: Translates simplified commands into low-level control signals.

5. **HTTP Bridge**: An API gateway that allows controlling the robot via web requests instead of direct ROS messages.

## Running in Simulation Mode

```bash
# Terminal 1: Start the Gazebo simulation
# This depends on your specific Gazebo launch file

# Terminal 2: Start the MCP interface
cd ~/xMo
source devel/setup.bash
rosrun svan_simple_control mcp.py

# Terminal 3: Run the simple control node
cd ~/xMo
source devel/setup.bash
rosrun svan_simple_control simulation.py

# Terminal 4: Run your control scripts or examples
cd ~/xMo
source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
```

## Running in Hardware Mode

```bash
# Complete all hardware startup steps first
# (joystick calibration, sleep calibration, moetus interface launch)

# Terminal 1: Start the MCP program
cd ~/dev/xMo
source devel/setup.bash
python3 mcp.py

# Terminal 2: Run the hardware interface
cd ~/dev/xMo
source devel/setup.bash
rosrun svan_simple_control hardware.py

# Terminal 3: Run your control script
cd ~/dev/xMo
source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
```

## Using the HTTP Bridge

The HTTP bridge provides a REST API to control the SVAN robot using HTTP requests instead of direct ROS messages.

### Setup and Running

```bash
# Create and activate a virtual environment
python3 -m venv bridge_venv
source bridge_venv/bin/activate

# Install required packages
pip install -r bridge/requirements.txt

# Start the bridge
python3 bridge/bridge.py
```

The bridge will start on port 8888. You can access:
- API endpoints at `http://127.0.0.1:8888/`
- Interactive API documentation at `http://127.0.0.1:8888/docs`
- Command history at `http://127.0.0.1:8888/history`

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

For detailed instructions on using the HTTP bridge, see the [bridge/README.md](bridge/README.md) file.

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

For detailed instructions on using the HTTP bridge and its examples, see the [bridge/examples/README.md](bridge/examples/README.md) file.