# svan_simple_control


### setup
```bash
cd ~/xMo/src
git clone https://github.com/kry0sc0pic/svan_simple_control.git
git clone https://github.com/kry0sc0pic/svan_simple_control_msgs.git
cd ~/xMo
echo "catkin make svan_simple_control_msgs" >> svan_build.sh
echo "source devel/setup.bash" >> svan_build.sh
echo "catkin make svan_simple_control" >> svan_build.sh
echo "source devel/setup.bash" >> svan_build.sh
./svan_build
```

### running
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

### message definition
these are the following variables you can give as part of the `SvanCommand` message. You can view the source [here](https://github.com/kry0sc0pic/svan_simple_control_msgs)

`command_type` (`int8`) - what aspect you want to control

| value | aspect |
| ---- | --- |
| 0 | OPERATION MODE (trot, push, up, etc.) |
| 1 | LINEAR MOVEMENT |
| 2 | ROLL ANGLE |
| 3 | PITCH ANGLE | 
| 4 | YAW VELOCITY |
| 5 | HEIGHT |

//TODO: complete docs


### examples

**sine wave circle**
<video src="examples/videos/sine_wave_circle.mov">