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
rosrun svan_simple_control simulation.py
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
> Always be ready to take manual control of the SVAN using the joystick incase something unexpected starts happening. 
>
> After the node starts, if some joystick data is published. It will cease sending commands until the node is restarted.
> 
> The simple control node differentiates between itself and the joystick command code by publishing a value of `1000.0` at index `7`. If the value is not `1000.0` (in case of the joystick command node) it ceases sending any recieved commands.
>
> Add the code from `handoff.txt` to the source on the joystick commander to prevent operation mode desyncs like [this](https://x.com/0xkry0sc0pic/status/1908403919379640780).


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
The HTTP bridge allows sending commands to the SVAN using POST requests to remove the ros dependency for the client machine or work around ROS connection issues.

The documentation can be accessed by running the bridge with `rosrun svan_simple_control bridge.py` and navigating to `/docs` in a web browser. The parameter names correspond to the [ROS Message Definition](https://github.com/kry0sc0pic/svan_simple_control_msgs) below.


## message definitions
these are the following variables you can give as part of the `SvanCommand` message. You can view the source [here](https://github.com/kry0sc0pic/svan_simple_control_msgs)

0. `command_type` (`uint8`) - what aspect you want to control

    | value | aspect |
    | ---- | --- |
    | 0 | OPERATION MODE (trot, push, up, etc.) |
    | 1 | LINEAR MOVEMENT |
    | 2 | ROLL ANGLE |
    | 3 | PITCH ANGLE | 
    | 4 | YAW VELOCITY |
    | 5 | HEIGHT |

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

    _granular height control is being actively developled_

3. `vel_x` (`float32`) (`-1.0` - `1.0`) - normalised value of velocity in x-axis. postive x-axis is the right of the robot.

4. `vel_y` (`float32`) (`-1.0` - `1.0`) - normalised value of velcity in y-axis. positive y-axis is the front of the robot.

5. `roll` (`float32`) - normalised roll angle. used when `command_type` is `2`.

    | value | roll angle |
    | --- | --- |
    | -1 | LEFT |
    | 0 | CENTER |
    | 1 | RIGHT |

6. `pitch` (`float32`) - normalised pitch angle. used when command_type is `3`.

    | value | roll angle |
    | --- | --- |
    | 1 | FRONT |
    | 0 | CENTER |
    | -1 | BACK |

7. `yaw` (`uint8`) - yaw direction. used when command_type is `4`.

    | value | direction |
    | --- | --- |
    | 0 | LEFT |
    | 1 | RIGHT |
    | 2 | NONE |



## examples
*bridge versions of certain examples are also available. They can be identified with the `bridge` suffix in the name. They should be used with the HTTP bridge.*


### rotate
svan rotates around Z-axis.

source: `examples/rotate.py`

### pushup and twirl
svan rotates around Z-axis.

source: `examples/pushup_twirl.py`



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
