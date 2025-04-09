# SVAN Control HTTP Bridge

This bridge provides a REST API to control the SVAN robot using HTTP requests instead of direct ROS messages.

## Features

- REST API endpoints for all SVAN control commands
- JSON request/response format
- Interactive Swagger documentation at `/docs`
- Error handling and validation
- Mock mode for testing without ROS
- Command history tracking at `/history`

## Setup

### Prerequisites

- Ubuntu with ROS Noetic installed
- Python 3.6 or higher
- The SVAN robot packages installed in your ROS workspace

### Installation

1. Clone the repository:
```bash
git clone https://github.com/orionop/svan_simple_control.git
cd svan_simple_control
```

2. Create a virtual environment and install the required Python packages:
```bash
python3 -m venv bridge_venv
source bridge_venv/bin/activate
pip install -r bridge/requirements.txt
```

The requirements include:
- fastapi
- uvicorn
- pydantic
- requests
- PyYAML (for ROS integration)
- rospkg (for ROS integration)
- catkin_pkg (for ROS integration)

## Usage

### Starting the Bridge

1. Make sure ROS Core is running first in a separate terminal:
```bash
roscore
```

2. Make sure your ROS environment is sourced in a new terminal:
```bash
source ~/catkin_ws/devel/setup.bash  # For your catkin workspace
# OR
source ~/xMo/devel/setup.bash  # For simulation
# OR
source ~/dev/xMo/devel/setup.bash  # For hardware
```

3. Run the bridge:
```bash
source bridge_venv/bin/activate
python3 bridge/bridge.py
```

The server will start on port 8888. You can access the interactive API documentation at:
```
http://127.0.0.1:8888/docs
```

If ROS is not available, the bridge will run in "mock mode", where it will accept commands but not send them to ROS.

### How the Bridge Works

The bridge acts as an intermediary between HTTP clients and the ROS system:

1. **HTTP Server**: Accepts REST API requests from clients (web apps, mobile apps, scripts)
2. **Request Validation**: Validates and sanitizes incoming requests
3. **ROS Message Conversion**: Converts HTTP requests into proper ROS messages
4. **Publishing**: Publishes messages to the `/svan/simple_control` ROS topic
5. **Response Handling**: Returns success or error status to the client
6. **History Tracking**: Keeps track of commands for debugging

This allows you to control the SVAN robot from any programming language or environment that can make HTTP requests, without needing to implement ROS directly.

### API Endpoints

All endpoints accept POST requests with JSON bodies:

- `GET /` - Check if the bridge is running
- `GET /history` - View command history 
- `POST /mode` - Set operation mode
- `POST /movement` - Control linear movement
- `POST /roll` - Control roll angle
- `POST /pitch` - Control pitch angle
- `POST /yaw` - Control yaw direction
- `POST /height` - Control height

### Examples

#### Set operation mode to TROT:
```bash
curl -X POST http://127.0.0.1:8888/mode \
  -H "Content-Type: application/json" \
  -d '{"operation_mode": 4}'
```

#### Set movement (forward at half speed):
```bash
curl -X POST http://127.0.0.1:8888/movement \
  -H "Content-Type: application/json" \
  -d '{"vel_x": 0.0, "vel_y": 0.5}'
```

#### Stop the robot:
```bash
curl -X POST http://127.0.0.1:8888/mode \
  -H "Content-Type: application/json" \
  -d '{"operation_mode": 1}'
```

### Example Client

The repository includes an example client script to demonstrate how to use the API:

```bash
source bridge_venv/bin/activate
python3 bridge/example_client.py
```

## Testing Without ROS

If you want to test the API without a ROS environment, you can run the bridge directly and it will operate in mock mode:

```bash
source bridge_venv/bin/activate
python3 bridge/bridge.py
```

You can then check the command history at `/history` to verify your commands were received.

## Troubleshooting

- If you see "Unable to register with master node [http://localhost:11311]", it means roscore is not running. Start roscore in a separate terminal first.
- If commands aren't being received by the robot, check that the simple control node (simulation.py or hardware.py) is running.
- View the command history at `/history` to verify your commands are being received.
- For detailed logs, check the FastAPI server output. 