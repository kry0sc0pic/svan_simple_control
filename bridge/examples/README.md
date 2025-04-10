# SVAN HTTP Bridge Examples

These examples demonstrate how to control the SVAN robot using HTTP requests to the bridge API instead of direct ROS communication.

## Requirements

- Python 3.6 or higher
- `requests` library (installed via `pip install requests`)

## Available Examples

- `pushup_http.py` - Demonstrates a sequence of operation modes (Stop, Pushup, Twirl) using HTTP requests

## Running the Examples

### Step 1: Ensure the ROS environment is running

Follow the same setup procedure as in the main documentation for either Simulation or Hardware mode:
```bash
# For simulation
# Terminal 1: Start the Gazebo simulation
# Terminal 2: Start the MCP interface
# Terminal 3: Run the simple control node (simulation.py)

# For hardware
# Terminal 1: Start the MCP program
# Terminal 2: Run the hardware interface (hardware.py)
```

### Step 2: Start the Bridge

In a new terminal:
```bash
# Activate the virtual environment
source bridge_venv/bin/activate

# Start the bridge
python3 bridge/bridge.py
```

Verify the bridge is running by visiting http://127.0.0.1:8888/ in your browser.

### Step 3: Run the HTTP Example

In a new terminal:
```bash
# Activate the virtual environment (if needed)
source bridge_venv/bin/activate

# Run the example
python3 bridge/examples/pushup_http.py
```

## Key Differences from Direct ROS Examples

1. **No ROS Dependency**: These examples don't import ROS packages or initialize ROS nodes
2. **HTTP Communication**: Commands are sent via HTTP requests instead of ROS messages
3. **Platform Independence**: Can run on machines without ROS installed
4. **Error Handling**: Includes robust connection and error handling
5. **Command History**: Demonstrates how to retrieve command history from the bridge 