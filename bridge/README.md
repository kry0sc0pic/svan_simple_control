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

- ROS environment with the SVAN robot packages installed
- Python 3.6 or higher

### Installation

1. Clone this repository (if not already done):
```bash
git clone https://github.com/kry0sc0pic/svan_simple_control.git
cd svan_simple_control
```

2. Create a virtual environment and install the required Python packages:
```bash
python3 -m venv bridge_venv
source bridge_venv/bin/activate
pip install -r bridge/requirements.txt
```

## Usage

### Starting the Bridge

1. Make sure your ROS environment is sourced:
```bash
source ~/xMo/devel/setup.bash  # For simulation
# OR
source ~/dev/xMo/devel/setup.bash  # For hardware
```

2. Run the bridge:
```bash
source bridge_venv/bin/activate
python3 bridge/bridge.py
```

The server will start on port 8888. You can access the interactive API documentation at:
```
http://127.0.0.1:8888/docs
```

If ROS is not available, the bridge will run in "mock mode", where it will accept commands but not send them to ROS.

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

#### HTTP API Examples

The `bridge/examples/` directory contains Python scripts that demonstrate how to control the SVAN robot using the HTTP bridge:

- `pushup_http.py` - Execute a sequence of operation modes (Stop, Pushup, Twirl)

See `bridge/examples/README.md` for detailed instructions on running these examples.

#### Using curl

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

#### Using the Example Client

A more comprehensive example client is available:

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

## Integration with Other Systems

You can easily integrate with any system that supports HTTP requests, including:

- Web applications
- Mobile apps
- IoT devices
- Custom scripts

## Troubleshooting

- If you encounter connection errors, make sure the bridge is running on the correct port (8888).
- If commands aren't being received by the robot, check the API status to verify ROS is connected.
- View the command history at `/history` to verify your commands are being received.
- For detailed logs, check the FastAPI server output. 