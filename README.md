# svan_simple_control


## setup
```bash
# cd into workspace
cd ~/xMo/src

# clone packages
git clone https://github.com/kry0sc0pic/svan_simple_control.git
git clone https://github.com/kry0sc0pic/svan_simple_control_msgs.git

# add to build script
cd ~/xMo
echo "catkin build svan_simple_control_msgs" >> svan_build.sh
echo "source devel/setup.bash" >> svan_build.sh
echo "catkin build svan_simple_control" >> svan_build.sh
echo "source devel/setup.bash" >> svan_build.sh

# build workspace
./svan_build
```

## running
after starting the gazebo simulation launch file and the mcp.py file. run the following in a third terminal

```bash
cd ~/xMo
source devel/setup.bash
rosrun svan_simple_control svan_simple_control.py
```

_now open 4th terminal are run your script. replace `<script>` with the filename of the example you want to run._
```bash
cd ~/xMo
source devel/setup.bash
python3 src/svan_simple_control/examples/<script>.py
```

## message definition
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

2. `direction` (`uint8`) - direction of movement or yaw when setting velocity. used when `command_type` is `1` or `4`.

    ***Linear Movement (`command_type` 1)***
    | value | direction |
    | --- | --- |
    | 0 | FORWARD |
    | 1 | BACK |
    | 2 | LEFT |
    | 3 | RIGHT | 
    | 4 | NONE |

    _`NONE` direction is the equivalent of STOP operation mode. Using this removes the need to switch back to TROT mode after switching to STOP_

    ***Yaw Movement (`command_type` 4)***
    | value | direction |
    | --- | --- |
    | 0 | LEFT |
    | 1 | RIGHT |


3. `height` (`uint8`) - height profile to set. used when `command_type` is `5`.

    | value | height |
    | --- | --- |
    | 1 | UP (MAX) |
    | 2 | DOWN (MIN) |

    _granular height control is being actively developled_

4. `velocity` (`float32`) - normalised magnitude of velocity. used for linear movement and yaw commands (`command_type` is `1` or `4`). value is constrained to `0 - 1` range.

5. `roll` (`float32`) - normalised roll angle. used when `command_type` is `2`.

    | value | roll angle |
    | --- | --- |
    | 0 | LEFT |
    | 0.5 | CENTER |
    | 1 | RIGHT |

6. `pitch` (`float32`) - normalised pitch angle. used when command_type is `3`.

    | value | roll angle |
    | --- | --- |
    | 0 | FRONT |
    | 0.5 | CENTER |
    | 1 | BACK |

## examples

### sine wave circle
moves in a circular path while varying height.
source: `examples/sine_wave_circle.py`
<video controls src="https://github.com/kry0sc0pic/svan_simple_control/raw/refs/heads/main/examples/videos/sine_wave_circle.mov"></video>