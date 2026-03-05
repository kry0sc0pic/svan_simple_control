# svan_simple_control

A ROS package for high-level control of the SVAN quadruped robot. Provides a simple command interface over ROS topics, an HTTP bridge for clients that cannot connect to ROS directly, and a library of ready-to-run examples.

---

## Table of Contents

1. [Setup](#setup)
2. [Running](#running)
3. [HTTP Bridge](#http-bridge)
4. [Safety](#safety)
5. [Message Reference](#message-reference)
6. [Examples](#examples)
7. [Deprecated Examples](#deprecated-examples)

---

## Setup

### Simulation

```bash
cd ~/xMo/src
git clone https://github.com/kry0sc0pic/svan_simple_control.git

cd ~/xMo
echo "catkin build svan_simple_control" >> svan_build.sh
echo "source devel/setup.bash" >> svan_build.sh

catkin build svan_simple_control
source devel/setup.bash
```

### Hardware

```bash
cd ~/dev/xMo/src
git clone https://github.com/kry0sc0pic/svan_simple_control.git

cd ~/dev/xMo
echo "catkin build svan_simple_control" >> svan_build.sh
echo "source devel/setup.bash" >> svan_build.sh

catkin build svan_simple_control
source devel/setup.bash

# optional — required only if using the HTTP bridge
python3 -m pip install --upgrade pip
python3 -m pip install fastapi uvicorn pydantic
```

---

## Running

### Simulation

After starting the Gazebo simulation launch file and `mcp.py`, open a third terminal:

```bash
cd ~/xMo && source devel/setup.bash
rosrun svan_simple_control sim.py
```

Then open a fourth terminal to run an example script:

```bash
cd ~/xMo && source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
```

---

### Hardware (without bridge)

> If you are having issues subscribing and publishing to SVAN ROS topics from a desktop/laptop, use the [HTTP bridge](#http-bridge) instead.

After completing all startup steps (joystick calibration, sleep calibration, Moetus interface launch, `mcp.py`), open two SSH sessions on the SVAN.

**Session 1** — start the control node:
```bash
cd ~/dev/xMo && source devel/setup.bash
rosrun svan_simple_control hardware.py
```

**Session 2** — run your script:
```bash
cd ~/dev/xMo && source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
```

---

### Hardware (with bridge)

After completing all startup steps, open two SSH sessions on the SVAN.

**Session 1** — start the control node:
```bash
cd ~/dev/xMo && source devel/setup.bash
rosrun svan_simple_control hardware.py
```

**Session 2** — start the bridge:
```bash
cd ~/dev/xMo && source devel/setup.bash
rosrun svan_simple_control bridge.py
```

The bridge will be reachable from your laptop at `http://10.42.4.9:8888`. See the [HTTP Bridge](#http-bridge) section for details.

---

## Safety

> **Always be ready to take manual control of the SVAN using the joystick if something unexpected happens.**

The simple control node identifies its own messages by publishing `1000.0` at index `7` of the joystick data array. If it receives a message where index `7` is not `1000.0` (i.e. from the physical joystick commander), it treats it as a manual override and stops sending commands until the node is restarted.

| Context | Behaviour |
| --- | --- |
| `sim.py` | Override detection **disabled** by default (`DISABLE_MANUAL_OVERRIDE = True`). Set to `False` to enable. |
| `hardware.py` | Override detection **always active**. |

To prevent operation mode desyncs (example [here](https://x.com/0xkry0sc0pic/status/1908403919379640780)), apply the patch from `handoff.txt` to the joystick commander source on the hardware using `scripts/apply_handoff.py`.

---

## HTTP Bridge

The HTTP bridge exposes the SVAN control interface over a REST API, removing the ROS dependency from the client machine.

### Addresses

| Context | Base URL |
| --- | --- |
| Simulation (localhost) | `http://localhost:8888` |
| Hardware (on the SVAN) | `http://10.42.4.9:8888` |

> When using the bridge examples (`*_bridge.py`) or the API collections, update `--host` / `base_url` to match your context — `localhost` for simulation, `10.42.4.9` for hardware.

### API Documentation

Interactive docs are available at `<base_url>/docs` while the bridge is running.

### API Collections

Pre-built collections for Insomnia and Postman are in the `api/` directory:

| File | How to import |
| --- | --- |
| `api/insomnia_collection.json` | Insomnia → *Import / Export → Import Data* |
| `api/postman_collection.json` | Postman → *Import → Upload Files* |

Both collections define a `base_url` variable. Change it to `http://10.42.4.9:8888` when targeting hardware.

### Endpoints

| Method | Path | Description |
| --- | --- | --- |
| `GET` | `/` | Health check — returns bridge status and mock mode warning if ROS is not connected |
| `GET` | `/history` | List of all commands sent in the current session |
| `POST` | `/mode` | Set operation mode (`operation_mode`: 1–5) |
| `POST` | `/movement` | Set velocity (`vel_x`, `vel_y`: −1.0 to 1.0) |
| `POST` | `/roll` | Set roll angle (`roll`: −1.0 to 1.0) |
| `POST` | `/pitch` | Set pitch angle (`pitch`: −1.0 to 1.0) |
| `POST` | `/yaw` | Set yaw direction (`yaw`: 0=LEFT, 1=RIGHT, 2=NONE) |
| `POST` | `/height` | Set height state (`height`: 1=UP, 2=DOWN, 3=STOP) |

---

## Message Reference

Message type: `svan_simple_control/SvanCommand`. Source: `msg/SvanCommand.msg`.

### `command_type` (`uint8`)

Selects which aspect of the robot to control.

| Value | Constant | Aspect |
| --- | --- | --- |
| 0 | `COMMAND_OPERATION_MODE` | Operation mode |
| 1 | `COMMAND_MOVEMENT` | Linear velocity |
| 2 | `COMMAND_ROLL` | Roll angle |
| 3 | `COMMAND_PITCH` | Pitch angle |
| 4 | `COMMAND_YAW` | Yaw direction |
| 5 | `COMMAND_HEIGHT` | Height profile |
| 6 | `COMMAND_JOYSTICK_OVERRIDE` | _(unimplemented)_ |

---

### `operation_mode` (`uint8`)

Used when `command_type = 0`.

| Value | Constant | Mode |
| --- | --- | --- |
| 1 | `MODE_STOP` | Stop |
| 2 | `MODE_TWIRL` | Twirl |
| 3 | `MODE_PUSHUP` | Pushup |
| 4 | `MODE_TROT` | Trot |
| 5 | `MODE_SLEEP` | Sleep |

---

### `vel_x` / `vel_y` (`float32`, −1.0 to 1.0)

Used when `command_type = 1`.

| Field | Axis | Positive direction |
| --- | --- | --- |
| `vel_x` | Lateral | Right |
| `vel_y` | Longitudinal | Forward |

---

### `roll` (`float32`, −1.0 to 1.0)

Used when `command_type = 2`.

| Value | Position |
| --- | --- |
| −1.0 | Full left |
| 0.0 | Neutral |
| 1.0 | Full right |

---

### `pitch` (`float32`, −1.0 to 1.0)

Used when `command_type = 3`.

| Value | Position |
| --- | --- |
| 1.0 | Full forward |
| 0.0 | Neutral |
| −1.0 | Full back |

---

### `yaw` (`uint8`)

Used when `command_type = 4`.

| Value | Constant | Direction |
| --- | --- | --- |
| 0 | `YAW_LEFT` | Left |
| 1 | `YAW_RIGHT` | Right |
| 2 | `YAW_NONE` | Stop yawing |

---

### `height` (`uint8`)

Used when `command_type = 5`.

| Value | Constant | Behaviour |
| --- | --- | --- |
| 1 | `HEIGHT_UP` | Drive to maximum height |
| 2 | `HEIGHT_DOWN` | Drive to minimum height |
| 3 | `STOP_HEIGHT` | Hold current position |

> Granular height control is under active development.

---

## Examples

Bridge counterparts (identified by the `_bridge` suffix) send the same commands via HTTP and should be used with the [HTTP bridge](#http-bridge) instead of directly over ROS.

### Basic

| Example | Description | Bridge |
| --- | --- | --- |
| `examples/rotate.py` | Rotate continuously around the Z-axis | — |
| `examples/pushup_twirl.py` | Cycle through pushup and twirl modes | `examples/pushup_twirl_bridge.py` |
| `examples/wave.py` | Cycle height UP/DOWN — good first demo, no locomotion | `examples/wave_bridge.py` |
| `examples/strafe.py` | Diamond pattern using `vel_x` — primary strafe demo | `examples/strafe_bridge.py` |
| `examples/square.py` | Walk an approximate square using movement + yaw | `examples/square_bridge.py` |
| `examples/attitude_sweep.py` | Sweep roll and pitch through full range, robot stays in place | `examples/attitude_sweep_bridge.py` |

### Advanced

| Example | Description | Bridge |
| --- | --- | --- |
| `examples/patrol.py` | Indefinite back-and-forth patrol, Ctrl+C to stop cleanly | `examples/patrol_bridge.py` |
| `examples/choreography.py` | Scripted multi-act routine exercising the full API | `examples/choreography_bridge.py` |
| `examples/keyboard_teleop.py` | Real-time keyboard control (`w/s/a/d`, `q/e`, `r/f`, `1`–`5`) | `examples/keyboard_teleop_bridge.py` |
| `examples/gamepad.py` | USB gamepad control via pygame (left stick, face buttons, shoulders) | `examples/gamepad_bridge.py` |

**Prerequisites for interactive examples:**
```bash
pip install pynput   # keyboard_teleop
pip install pygame   # gamepad
```

---

## Deprecated Examples

These examples use the old `svan_simple_control_msgs` package and legacy API. They are kept for reference but will not run without modification.

| Example | Description | Author |
| --- | --- | --- |
| `examples/deprecated/sine_wave_circle.py` | Circular path with sinusoidal height variation | — |
| `examples/deprecated/hand_control.py` | Move/stop control via hand gestures (MediaPipe) | Atharv Nawale |
| `examples/deprecated/height_control.py` | Height control via hand gestures (MediaPipe) | Dhruv Shah |
