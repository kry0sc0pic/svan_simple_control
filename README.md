# svan_simple_control

## setup (simulation)
```bash
# cd into workspace
cd ~/xMo/src

# clone package
git clone https://github.com/kry0sc0pic/svan_simple_control.git
# add to build script
cd ~/xMo
echo "catkin build svan_simple_control" >> svan_build.sh
echo "source devel/setup.bash" >> svan_build.sh

# build package
catkin build svan_simple_control
source devel/setup.bash
```

## setup (hardware)
```bash
# cd into workspace
cd ~/dev/xMo/src

# clone packages
git clone https://github.com/kry0sc0pic/svan_simple_control.git

# add to build script
cd ~/dev/xMo
echo "catkin build svan_simple_control" >> svan_build.sh
echo "source devel/setup.bash" >> svan_build.sh

# build packages
catkin build svan_simple_control
source devel/setup.bash

# (optional) install dependencies for http bridge
python3 -m pip install --upgrade pip
python3 -m pip install fastapi uvicorn pydantic
```

## running (simulation)
after starting the gazebo simulation launch file and the mcp.py file. run the following in a third terminal

```bash
cd ~/xMo
source devel/setup.bash
rosrun svan_simple_control sim.py
```

_now open 4th terminal are run your script. replace `<script>` with the filename of the example you want to run._
```bash
cd ~/xMo
source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
```

---

> ***🔴 Important Note***
>
> Always be ready to take manual control of the SVAN using the joystick in case something unexpected starts happening.
>
> After the node starts, if joystick data is published by an external source, it will cease sending commands until the node is restarted.
>
> The simple control node differentiates between itself and the joystick command code by publishing a value of `1000.0` at index `7`. If the value is not `1000.0` (i.e. from the joystick commander node) it ceases sending any received commands.
>
> **Simulation (`sim.py`):** Manual override detection is disabled by default (`DISABLE_MANUAL_OVERRIDE = True`). Set this flag to `False` to enable it.
>
> **Hardware (`hardware.py`):** Manual override detection is always active.
>
> Apply the code from `handoff.txt` to the joystick commander source on the hardware using `scripts/apply_handoff.py` to prevent operation mode desyncs like [this](https://x.com/0xkry0sc0pic/status/1908403919379640780).


## running (hardware) (without bridge)

> if you are having issues subscribing and publishing to SVAN ros topics from a desktop/laptop. Have a look at the HTTP bridge below.

after completing all the startup steps (joystick calibration, sleep calibration, moetus interface launch and starting the `mcp.py`). run the following commands in two ssh sessions on the SVAN.

_in the first ssh session_
```bash
cd ~/dev/xMo
source devel/setup.bash
rosrun svan_simple_control hardware.py
```

_in the second ssh session_
```bash
cd ~/dev/xMo
source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
``` 

## running (hardware) (with bridge)

after completing all the startup steps (joystick calibration, sleep calibration, moetus interface launch and starting the `mcp.py`). run the following commands in two ssh sessions on the SVAN.

_in the first ssh session_
```bash
cd ~/dev/xMo
source devel/setup.bash
rosrun svan_simple_control hardware.py
```

_in the second ssh session_
```bash
cd ~/dev/xMo
source devel/setup.bash
rosrun svan_simple_control bridge.py
``` 

## bridge
The HTTP bridge allows sending commands to the SVAN using POST requests to remove the ROS dependency for the client machine or work around ROS connection issues.

The documentation can be accessed by running the bridge with `rosrun svan_simple_control bridge.py` and navigating to `/docs` in a web browser. The parameter names correspond to the message definition below.

API collections for importing into Insomnia or Postman are available in the `api/` directory:

- `api/insomnia_collection.json` — import via Insomnia → *Import / Export → Import Data*
- `api/postman_collection.json` — import via Postman → *Import → Upload Files*

Both collections use a `base_url` variable (default `http://localhost:8888`) that you can override per-environment.


## message definitions
these are the following variables you can give as part of the `SvanCommand` message. The source is at `msg/SvanCommand.msg`.

0. `command_type` (`uint8`) - what aspect you want to control

    | value | aspect |
    | ---- | --- |
    | 0 | OPERATION MODE (trot, pushup, etc.) |
    | 1 | LINEAR MOVEMENT |
    | 2 | ROLL ANGLE |
    | 3 | PITCH ANGLE |
    | 4 | YAW DIRECTION |
    | 5 | HEIGHT |
    | 6 | JOYSTICK OVERRIDE _(unimplemented)_ |

1. `operation_mode` (`uint8`) - operation mode to switch to. used when `command_type` is `0`.

    | value | mode |
    | --- | --- |
    | 1 | STOP |
    | 2 | TWIRL |
    | 3 | PUSHUP |
    | 4 | TROT |
    | 5 | SLEEP |


2. `height` (`uint8`) - height profile to set. used when `command_type` is `5`.

    | value | height |
    | --- | --- |
    | 1 | UP (MAX) |
    | 2 | DOWN (MIN) |
    | 3 | STOP (hold current position) |

    _granular height control is being actively developed_

3. `vel_x` (`float32`) (`-1.0` - `1.0`) - normalised value of velocity in x-axis. positive x-axis is the right of the robot.

4. `vel_y` (`float32`) (`-1.0` - `1.0`) - normalised value of velocity in y-axis. positive y-axis is the front of the robot.

5. `roll` (`float32`) - normalised roll angle. used when `command_type` is `2`.

    | value | roll angle |
    | --- | --- |
    | -1 | LEFT |
    | 0 | CENTER |
    | 1 | RIGHT |

6. `pitch` (`float32`) - normalised pitch angle. used when `command_type` is `3`.

    | value | pitch angle |
    | --- | --- |
    | 1 | FRONT |
    | 0 | CENTER |
    | -1 | BACK |

7. `yaw` (`uint8`) - yaw direction. used when `command_type` is `4`.

    | value | direction |
    | --- | --- |
    | 0 | LEFT |
    | 1 | RIGHT |
    | 2 | NONE |



## examples
*bridge versions of certain examples are also available. They can be identified with the `bridge` suffix in the name. They should be used with the HTTP bridge.*

### rotate
SVAN rotates continuously around the Z-axis.

source: `examples/rotate.py`

### pushup and twirl
SVAN cycles through pushup and twirl modes.

source: `examples/pushup_twirl.py` | bridge: `examples/pushup_twirl_bridge.py`

### wave
Cycles the robot's height between UP and DOWN repeatedly, creating a bowing/waving motion. Good first demo — no locomotion required.

source: `examples/wave.py` | bridge: `examples/wave_bridge.py`

### square
Walks the robot in an approximate square: forward, turn 90° right, repeat four times. Demonstrates combining movement and yaw in a structured sequence.

Tune `SIDE_SECS` and `TURN_SECS` at the top of the file to match your surface and speed.

source: `examples/square.py` | bridge: `examples/square_bridge.py`

### strafe
Demonstrates lateral (`vel_x`) movement by walking a diamond pattern: forward → strafe right → backward → strafe left. The primary example for `vel_x`.

source: `examples/strafe.py` | bridge: `examples/strafe_bridge.py`

### attitude sweep
Sweeps roll and pitch independently through their full range (−1 → 0 → 1 → 0), demonstrating the body orientation API. The robot stays in place — no locomotion.

source: `examples/attitude_sweep.py` | bridge: `examples/attitude_sweep_bridge.py`

### patrol
Runs an indefinite back-and-forth patrol: walk forward for `LEG_SECS`, stop briefly, turn 180°, repeat. Press Ctrl+C to stop cleanly. Useful for continuous lab demos and testing sustained locomotion.

source: `examples/patrol.py` | bridge: `examples/patrol_bridge.py`

### choreography
A scripted multi-step routine that exercises the full API: walk in → height wave → attitude display → twirl → pushup sequence → walk out → sleep. Introduces a `play_sequence()` helper for composing routines from `(SvanCommand, hold_seconds)` tuples.

source: `examples/choreography.py` | bridge: `examples/choreography_bridge.py`

### keyboard teleop
Real-time keyboard control over ROS. `w/s/a/d` to move, `q/e` to yaw, `r/f` for height, `1`–`5` for modes, Space to halt, Esc to quit.

**Requires:** `pip install pynput`

source: `examples/keyboard_teleop.py` | bridge: `examples/keyboard_teleop_bridge.py`

### gamepad
USB gamepad control over ROS using pygame. Left stick for movement, right stick X for yaw, face buttons for modes, shoulder buttons for height.

**Requires:** `pip install pygame`

source: `examples/gamepad.py` | bridge: `examples/gamepad_bridge.py`



## examples (deprecated)

### sine wave circle

moves in a circular path while varying height.

source: `examples/deprecated/sine_wave_circle.py`

### hand gesture movement control

author: [Atharv Nawale]()

move and stop the svan using hand gestures. powered by mediapipe.

source: `examples/deprecated/hand_control.py`

### hand gesture height control

author: [Dhruv Shah]()

control the svan's height using gestures. powered by mediapipe.

source: `examples/deprecated/height_control.py`
