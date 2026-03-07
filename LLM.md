# svan_simple_control

A ROS package providing high-level control of the SVAN quadruped robot. Offers a simple command interface over ROS topics, an HTTP bridge for non-ROS clients, and utilities for hardware calibration.

---

## Directory Structure

```
svan_simple_control/
├── src/
│   ├── hardware.py      # Hardware control node
│   ├── sim.py           # Simulation control node
│   └── bridge.py        # FastAPI HTTP bridge
├── msg/
│   └── SvanCommand.msg # ROS message definition
├── examples/            # Example control scripts
├── launch/
│   ├── simple_control.launch       # Hardware launch
│   └── simple_control_bridged.launch
├── config/
│   └── hardware_offsets.yaml      # Drift correction
├── scripts/
│   ├── tune_offsets.py            # Interactive offset tuner
│   └── apply_handoff.py           # State sync patch applier
├── api/
│   ├── postman_collection.json
│   └── insomnia_collection.json
├── CMakeLists.txt
├── package.xml
└── bridge-requirements.txt
```

---

## Message Reference

**Topic**: `/svan/simple_control`

**Type**: `svan_simple_control/SvanCommand`

### Constants

#### Operation Modes (`uint8`)
| Value | Constant | Mode |
|-------|----------|------|
| 1 | `MODE_STOP` | Stop |
| 2 | `MODE_TWIRL` | Twirl |
| 3 | `MODE_PUSHUP` | Pushup |
| 4 | `MODE_TROT` | Trot |
| 5 | `MODE_SLEEP` | Sleep |

#### Command Type (`uint8`)
| Value | Constant | Aspect |
|-------|----------|--------|
| 0 | `COMMAND_OPERATION_MODE` | Operation mode |
| 1 | `COMMAND_MOVEMENT` | Linear velocity |
| 2 | `COMMAND_ROLL` | Roll angle |
| 3 | `COMMAND_PITCH` | Pitch angle |
| 4 | `COMMAND_YAW` | Yaw direction |
| 5 | `COMMAND_HEIGHT` | Height profile |
| 6 | `COMMAND_JOYSTICK_OVERRIDE` | (unimplemented) |

#### Height (`uint8`)
| Value | Constant | Behaviour |
|-------|----------|-----------|
| 1 | `HEIGHT_UP` | Drive to maximum height |
| 2 | `HEIGHT_DOWN` | Drive to minimum height |
| 3 | `STOP_HEIGHT` | Hold current position |

#### Yaw Direction (`uint8`)
| Value | Constant | Direction |
|-------|----------|-----------|
| 0 | `YAW_LEFT` | Left |
| 1 | `YAW_RIGHT` | Right |
| 2 | `YAW_NONE` | Stop yawing |

### Fields

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `command_type` | `uint8` | — | Which aspect to control (see constants above) |
| `operation_mode` | `uint8` | 1–5 | Target operation mode |
| `vel_x` | `float32` | −1.0 to 1.0 | Lateral velocity (positive = right) |
| `vel_y` | `float32` | −1.0 to 1.0 | Longitudinal velocity (positive = forward) |
| `roll` | `float32` | −1.0 to 1.0 | Roll angle (−1 = full left, 0 = neutral, +1 = full right) |
| `pitch` | `float32` | −1.0 to 1.0 | Pitch angle (+1 = full forward, 0 = neutral, −1 = full back) |
| `yaw` | `uint8` | 0–2 | Yaw direction |
| `height` | `uint8` | 1–3 | Height state |

---

## Core Nodes

### hardware.py

Hardware control node for the physical SVAN robot.

**Subscribes**: `/svan/simple_control` (SvanCommand)

**Publishes**: `/svan/joystick_data` (Float32MultiArray)

**Key Features**:

- **Manual Override Detection**: Subscribes to `/svan/joystick_data`. When index 7 is not `1000.0`, treats it as manual joystick input and triggers failsafe, ignoring all subsequent commands until node restart.
- **Drift Correction**: Loads `offset_velocity_x` and `offset_velocity_y` from `config/hardware_offsets.yaml` and applies them to every movement command to counteract surface bias.
- **State Machine**: Maintains current operation mode internally and constructs appropriate joystick data arrays for the robot.

**ROS Launch**: `rosrun svan_simple_control hardware.py`

---

### sim.py

Simulation control node for Gazebo or similar simulators.

**Subscribes**: `/svan/simple_control` (SvanCommand)

**Publishes**: `/svan/io_interface` (Float32MultiArray)

**Key Features**:

- **Manual Override**: Controlled by `DISABLE_MANUAL_OVERRIDE` constant (default: `True`). When `False`, subscribes to `/svan/io_interface` and detects override the same way hardware.py does.
- **No Drift Correction**: Publishes commands directly without offset application.

**ROS Launch**: `rosrun svan_simple_control sim.py`

---

### bridge.py

FastAPI-based HTTP bridge exposing the control interface over REST.

**ROS Integration**: Publishes to `/svan/simple_control` and subscribes to `/svan/joystick_data` for override detection.

**Key Features**:

- **Mock Mode**: If ROS initialization fails (e.g., no roscore), runs in mock mode—commands are logged but not sent to ROS. Health check endpoint returns warning status.
- **Command History**: Stores all commands in memory, accessible via `/history` endpoint.

**Dependencies**: fastapi, uvicorn, pydantic

**Run**: `rosrun svan_simple_control bridge.py`

**Default URL**: `http://0.0.0.0:8888`

---

## HTTP Bridge API

**Base URL**: `http://localhost:8888` (simulation) or `http://10.42.4.9:8888` (hardware)

Interactive docs available at `<base_url>/docs`

### Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | Health check |
| `GET` | `/history` | Command history |
| `POST` | `/mode` | Set operation mode |
| `POST` | `/movement` | Set velocity |
| `POST` | `/roll` | Set roll angle |
| `POST` | `/pitch` | Set pitch angle |
| `POST` | `/yaw` | Set yaw direction |
| `POST` | `/height` | Set height state |

### Request/Response Formats

#### GET /

**Response**:
```json
{
  "status": "ok" | "warning",
  "message": "string"
}
```
`warning` status indicates mock mode (ROS not connected).

---

#### GET /history

**Response**:
```json
{
  "status": "ok",
  "commands": [
    {
      "command_type": "string",
      // ... fields specific to command type
    }
  ]
}
```

---

#### POST /mode

**Request**:
```json
{
  "operation_mode": 1 | 2 | 3 | 4 | 5
}
```

**Response**:
```json
{
  "status": "ok" | "mock",
  "message": "string"
}
```

---

#### POST /movement

**Request**:
```json
{
  "vel_x": float,  // -1.0 to 1.0
  "vel_y": float   // -1.0 to 1.0
}
```

Values are automatically constrained to valid range.

**Response**:
```json
{
  "status": "ok" | "mock",
  "message": "string"
}
```

---

#### POST /roll

**Request**:
```json
{
  "roll": float  // -1.0 to 1.0
}
```

**Response**:
```json
{
  "status": "ok" | "mock",
  "message": "string"
}
```

---

#### POST /pitch

**Request**:
```json
{
  "pitch": float  // -1.0 to 1.0
}
```

**Response**:
```json
{
  "status": "ok" | "mock",
  "message": "string"
}
```

---

#### POST /yaw

**Request**:
```json
{
  "yaw": 0 | 1 | 2
}
```

**Response**:
```json
{
  "status": "ok" | "mock",
  "message": "string"
}
```

---

#### POST /height

**Request**:
```json
{
  "height": 1 | 2 | 3
}
```

**Response**:
```json
{
  "status": "ok" | "mock",
  "message": "string"
}
```

---

## Configuration

### hardware_offsets.yaml

Located at: `config/hardware_offsets.yaml`

**Format**:
```yaml
offset_velocity_x: float   # lateral axis correction (-1.0 to 1.0)
offset_velocity_y: float   # longitudinal axis correction (-1.0 to 1.0)
```

**Purpose**: Corrects for systematic drift when the robot tends to veer on a specific surface.

**Loading**: hardware.py reads this file on startup. No rebuild required—restart the node to apply changes.

---

## Scripts

### tune_offsets.py

Interactive terminal tool for tuning drift-correction offsets.

**Usage**: `python3 scripts/tune_offsets.py`

**Controls**:
| Key | Action |
|-----|--------|
| Up/Down Arrow | Adjust `offset_velocity_y` (longitudinal) |
| Left/Right Arrow | Adjust `offset_velocity_x` (lateral) |
| `r` | Reset both to 0.0 |
| `s` | Save current values |
| `q` / `Esc` / `Ctrl+C` | Save and quit |

Changes are saved to `config/hardware_offsets.yaml`. Step size: 0.001 per keypress.

---

### apply_handoff.py

Applies the state-sync patch from `handoff.txt` to the SVAN joystick commander source.

**Purpose**: Prevents operation mode desync between the joystick commander and this control package. Without this patch, both sources can send conflicting mode commands, causing unpredictable robot behavior.

**Usage**:
1. Edit `apply_handoff.py` and set `TARGET_FILE` to the absolute path of the joystick commander Python source.
2. Run: `python3 scripts/apply_handoff.py`

**What the patch does**: Subscribes to `/svan/simple_control` and updates the joystick data array's operation mode index whenever a command with `COMMAND_OPERATION_MODE` is received, ensuring the joystick commander sees the same mode state.

---

## Known Limitations

1. **Granular Height Control**: Only discrete height commands are supported (UP/DOWN/STOP). Continuous height positioning is under active development.

2. **COMMAND_JOYSTICK_OVERRIDE**: Defined in the message but not implemented in any node.

3. **Yaw Requires Non-Zero Velocity**: The robot's firmware requires a small forward velocity for yaw commands to take effect. All examples and the bridge send a tiny nudge (`vel_y = 0.0001`) alongside yaw commands.

4. **State Sync Requirement**: On hardware, mode desync can occur between the joystick commander and this package. The `apply_handoff.py` patch is required to prevent this.

5. **Bridge Connectivity**: HTTP bridge must run on the same machine as ROS (the robot). Clients connect over the network but cannot run the bridge themselves.

6. **Override Failsafe is Permanent**: Once triggered by manual joystick input, the failsafe remains active until the hardware.py node is restarted. There is no automatic recovery.

---

## Troubleshooting

### "Manual Override" / Failsafe Triggered

**Symptom**: Node logs "Manual Override" and stops responding to commands.

**Cause**: Physical joystick sent input while this package was running.

**Fix**: Restart the hardware.py node (`Ctrl+C` and re-run `rosrun svan_simple_control hardware.py`).

**Prevention**: Ensure only one control source is active. Use the joystick OR this package, not both simultaneously.

---

### Cannot Connect to Bridge

**Symptom**: `ConnectionError` when accessing `http://10.42.4.9:8888`

**Cause**: Bridge is not running, or wrong IP address.

**Fix**:
1. Verify bridge is running: `rosrun svan_simple_control bridge.py` on the robot
2. Check IP: Ensure `10.42.4.9` is the robot's correct IP address
3. Check network connectivity: `ping 10.42.4.9`

---

### Bridge Running in Mock Mode

**Symptom**: Health check returns `"status": "warning"` with mock mode message.

**Cause**: Bridge cannot connect to ROS (no roscore, or not sourced correctly).

**Fix**:
1. Ensure ROS is running: `roscore`
2. Source the workspace: `source devel/setup.bash`
3. Restart the bridge

---

### Mode Desync (Robot Behaves Unexpectedly)

**Symptom**: Robot switches modes unexpectedly or ignores commands.

**Cause**: Both joystick commander and this package sending mode commands without state synchronization.

**Fix**: Apply the handoff patch:
```bash
python3 scripts/apply_handoff.py
```
Then restart the joystick commander node.

---

### Robot Drifts to One Side

**Symptom**: Robot consistently curves left or right when commanded to move straight.

**Cause**: Mechanical bias on the current surface.

**Fix**: Run the offset tuner:
```bash
python3 scripts/tune_offsets.py
```
Adjust offsets until the robot drives straight. Save and restart hardware.py.

---

### Gamepad Not Detected

**Symptom**: `gamepad.py` exits with "No gamepad detected"

**Cause**: No USB gamepad connected, or pygame joystick initialization failed.

**Fix**:
1. Connect a USB gamepad
2. Verify it's recognized: `python3 -c "import pygame; pygame.joystick.init(); print(pygame.joystick.get_count())"`
3. Try a different USB port

---

### Keyboard Teleop Not Responding

**Symptom**: Pressing keys produces no output or robot movement.

**Cause**: Missing `pynput` dependency, or focus not on terminal window.

**Fix**:
1. Install dependency: `pip install pynput`
2. Click the terminal window to ensure it has keyboard focus
3. Check for system permission issues (macOS may require accessibility permissions)

---

### Permission Denied When Running Scripts

**Symptom**: `Permission denied` when running `rosrun svan_simple_control sim.py`

**Cause**: Script files may not have execute permission.

**Fix**:
```bash
chmod +x src/sim.py src/hardware.py src/bridge.py
```

---

### No Movement When Yaw Command Sent

**Symptom**: Yaw command is sent but robot does not turn.

**Cause**: Yaw requires non-zero velocity to take effect.

**Fix**: Ensure yaw commands are accompanied by a small forward velocity (see all examples for the correct pattern):
```python
pub_yaw(SvanCommand.YAW_LEFT)
pub_movement(0.0, 0.0001)  # tiny nudge
```

---

### Example: Creating a Custom Command

To create a custom example or integrate this package into another application, the pattern is:

1. Import the message: `from svan_simple_control.msg import SvanCommand`
2. Create a publisher to `/svan/simple_control`
3. Construct SvanCommand objects with appropriate `command_type` and target field
4. Publish the command

All values are normalized (−1.0 to 1.0 for velocities/angles, discrete constants for modes/heights).
