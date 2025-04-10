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

1. Clone both repositories (if not already done):
```bash
# Navigate to your ROS workspace source directory
cd ~/xMo/src

# Clone the main control repository
git clone https://github.com/kry0sc0pic/svan_simple_control.git

# Clone the messages repository (required for the messages definitions)
git clone https://github.com/kry0sc0pic/svan_simple_control_msgs.git

# Build the packages
cd ~/xMo
catkin build svan_simple_control_msgs
source devel/setup.bash
catkin build svan_simple_control
source devel/setup.bash
```

2. Create a virtual environment and install the required Python packages:
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

## Running the System with HTTP Bridge

You'll need 6 terminals to run the complete system:

### Terminal 1: ROS Core
```bash
# Start ROS Core
roscore
```

### Terminal 2: Gazebo Simulation (for simulation mode only)
```bash
# Navigate to your workspace
cd ~/xMo

# Source the ROS environment
source devel/setup.bash

# Launch Gazebo simulation
roslaunch svan_bringup gazebo_sim.launch
```

### Terminal 3: MCP Interface
```bash
# Navigate to your workspace
cd ~/xMo

# Source the ROS environment
source devel/setup.bash

# Run the MCP program
python3 src/svan_control/src/mcp.py
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

When the bridge starts, you should see output like:
```
Starting SVAN Control Bridge
Access the API at http://127.0.0.1:8888
Access the docs at http://127.0.0.1:8888/docs
View command history at http://127.0.0.1:8888/history
```

### Terminal 6: Run HTTP Examples
```bash
# Navigate to the svan_simple_control directory
cd ~/xMo/src/svan_simple_control

# Activate the virtual environment
source bridge_venv/bin/activate

# Run the example
python3 bridge/examples/pushup_http.py
```

## API Endpoints

All endpoints accept POST requests with JSON bodies:

- `GET /` - Check if the bridge is running
- `GET /history` - View command history 
- `POST /mode` - Set operation mode
- `POST /movement` - Control linear movement
- `POST /roll` - Control roll angle
- `POST /pitch` - Control pitch angle
- `POST /yaw` - Control yaw direction
- `POST /height` - Control height

## Examples

### HTTP API Examples

The bridge provides example Python scripts that demonstrate how to control the SVAN robot using the HTTP bridge:

- `bridge/examples/pushup_http.py` - Execute a sequence of operation modes (Stop, Pushup, Twirl)

To run the example:
```bash
cd ~/xMo/src/svan_simple_control
source bridge_venv/bin/activate
python3 bridge/examples/pushup_http.py
```

### Using curl

You can also test the API directly using curl:

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

### Using the Example Client

A more comprehensive example client is available:

```bash
cd ~/xMo/src/svan_simple_control
source bridge_venv/bin/activate
python3 bridge/example_client.py
```

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