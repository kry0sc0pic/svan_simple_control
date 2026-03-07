#!/usr/bin/env python3
import os
import yaml
from svan_simple_control.msg import SvanCommand
from std_msgs.msg import Float32MultiArray
import rospy

rospy.init_node("svan_simple_control_node")

command_publisher = rospy.Publisher(
    "/svan/joystick_data", Float32MultiArray, queue_size=10
)
failsafe = False

# ── Manual override deadzone ───────────────────────────────────────────────────
# Minimum absolute change required in any field before an incoming joystick
# message is considered a genuine external override.  This prevents noisy
# near-zero signals from accidentally tripping the failsafe.
#
#   OVERRIDE_IDENTITY_DEADZONE  – applied to index 7 (the sentinel field).
#       Our messages stamp index 7 with 1000.0; an external message will be
#       close to 0.  The gap is large, so a generous threshold is fine.
#   OVERRIDE_AXIS_DEADZONE      – applied to all other indices (0-6, 8).
#       Joystick axes idle near 0; a 0.03 threshold filters electrical noise
#       while still catching any intentional stick movement.
OVERRIDE_IDENTITY_DEADZONE: float = 1.0
OVERRIDE_AXIS_DEADZONE: float = 0.03
_last_override_msg: list = []

# Load drift-correction offsets from config/hardware_offsets.yaml.
# Fall back to 0.0 / 0.0 with a warning if the file is missing or malformed.
_OFFSETS_PATH = os.path.join(
    os.path.dirname(__file__), "..", "config", "hardware_offsets.yaml"
)
try:
    with open(_OFFSETS_PATH, "r") as _f:
        _offsets = yaml.safe_load(_f) or {}
    OFFSET_VELOCITY_X = float(_offsets.get("offset_velocity_x", 0.0))
    OFFSET_VELOCITY_Y = float(_offsets.get("offset_velocity_y", 0.0))
    rospy.loginfo(
        f"Loaded hardware offsets: OFFSET_VELOCITY_X={OFFSET_VELOCITY_X}, "
        f"OFFSET_VELOCITY_Y={OFFSET_VELOCITY_Y}"
    )
except Exception as _e:
    rospy.logwarn(
        f"Could not load hardware_offsets.yaml ({_e}); defaulting offsets to 0.0"
    )
    OFFSET_VELOCITY_X = 0.0
    OFFSET_VELOCITY_Y = 0.0


def override_listener(msg: Float32MultiArray):
    global failsafe, _last_override_msg

    if failsafe:
        return

    data = list(msg.data)

    # First message — record it and wait for a change before deciding.
    if not _last_override_msg:
        _last_override_msg = data
        return

    # Identity check: is index 7 meaningfully different from our 1000-sentinel?
    identity_changed = abs(data[7] - 1000.0) > OVERRIDE_IDENTITY_DEADZONE

    # Change check: did any field move more than the axis deadzone?
    axis_changed = any(
        abs(data[i] - _last_override_msg[i]) > OVERRIDE_AXIS_DEADZONE
        for i in range(len(data))
        if i != 7
    )

    _last_override_msg = data

    if identity_changed and axis_changed:
        rospy.logerr("Manual Override")
        failsafe = True


base_data = [0, 0, 0, 0, 0, 0, 0, 1000, 0]
current_operation_mode = SvanCommand.MODE_STOP
current_joystick_data = Float32MultiArray()
current_joystick_data.data = base_data


def constrain_value(value, minumum: float = -1.0, maximum: float = 1.0):
    return max(minumum, min(value, maximum))


def set_operation_mode(mode: int):
    global current_operation_mode, current_joystick_data, command_publisher
    current_joystick_data.data = list(base_data)
    if mode == SvanCommand.MODE_TROT:
        rospy.loginfo("Trot Mode")
        current_joystick_data.data[0] = 4.0
        current_operation_mode = SvanCommand.MODE_TROT
    elif mode == SvanCommand.MODE_PUSHUP:
        rospy.loginfo("Pushup Mode")
        current_operation_mode = SvanCommand.MODE_PUSHUP
        current_joystick_data.data[0] = 3.0
    elif mode == SvanCommand.MODE_TWIRL:
        rospy.loginfo("Twirl Mode")
        current_operation_mode = SvanCommand.MODE_TWIRL
        current_joystick_data.data[0] = 2.0
    elif mode == SvanCommand.MODE_STOP:
        rospy.loginfo("Stop Mode")
        current_operation_mode = SvanCommand.MODE_STOP
        current_joystick_data.data[0] = 1.0
    elif mode == SvanCommand.MODE_SLEEP:
        rospy.loginfo("Sleep Mode")
        current_operation_mode = SvanCommand.MODE_SLEEP
        current_joystick_data.data[0] = 6.0
    command_publisher.publish(current_joystick_data)


def set_velocity(vel_x: float = 0.0, vel_y: float = 0.0):
    global current_operation_mode, current_joystick_data
    if vel_x == 0.0 and vel_y == 0.0:
        if current_operation_mode == SvanCommand.MODE_TROT:
            set_operation_mode(SvanCommand.MODE_STOP)
        return

    if current_operation_mode != SvanCommand.MODE_TROT:
        set_operation_mode(SvanCommand.MODE_TROT)

    vel_x = constrain_value(vel_x + OFFSET_VELOCITY_X)
    vel_y = constrain_value(vel_y + OFFSET_VELOCITY_Y)
    rospy.loginfo(f"Velocity: Vel_X: {vel_x} Vel_Y: {vel_y}")
    current_joystick_data.data[1] = vel_x
    current_joystick_data.data[2] = vel_y
    command_publisher.publish(current_joystick_data)


def set_roll(magnitude: float):
    global current_joystick_data
    magnitude = constrain_value(magnitude)
    current_joystick_data.data[3] = magnitude
    rospy.loginfo(f"Roll: {magnitude}")
    command_publisher.publish(current_joystick_data)


def set_pitch(magnitude: float):
    global current_joystick_data
    magnitude = constrain_value(magnitude)
    current_joystick_data.data[4] = magnitude
    rospy.loginfo(f"Pitch: {magnitude}")
    command_publisher.publish(current_joystick_data)


def set_yaw(direction: int = SvanCommand.YAW_NONE):
    global current_joystick_data
    if direction == SvanCommand.YAW_NONE:
        current_joystick_data.data[5] = 0.0
        current_joystick_data.data[6] = 0.0
        rospy.loginfo("Yaw: None")
        command_publisher.publish(current_joystick_data)

    elif direction == SvanCommand.YAW_RIGHT:
        current_joystick_data.data[5] = 1.0
        current_joystick_data.data[6] = 0.0
        rospy.loginfo("Yaw: Right")
        command_publisher.publish(current_joystick_data)

    elif direction == SvanCommand.YAW_LEFT:
        current_joystick_data.data[5] = 0.0
        current_joystick_data.data[6] = 1.0
        rospy.loginfo("Yaw: Left")
        command_publisher.publish(current_joystick_data)


def set_height(state: int):
    global current_joystick_data

    if state == SvanCommand.HEIGHT_UP and current_joystick_data.data[8] != 1:
        current_joystick_data.data[8] = 1.0
        rospy.loginfo("Height: Up")
        command_publisher.publish(current_joystick_data)

    elif state == SvanCommand.HEIGHT_DOWN and current_joystick_data.data[8] != -1:
        current_joystick_data.data[8] = -1.0
        rospy.loginfo("Height: Down")
        command_publisher.publish(current_joystick_data)

    elif state == SvanCommand.STOP_HEIGHT and current_joystick_data.data[8] != 0:
        current_joystick_data.data[8] = 0.0
        rospy.loginfo("Height: Stop")
        command_publisher.publish(current_joystick_data)


def handle_new_command(command: SvanCommand):
    global failsafe
    rospy.logdebug(f"Recieved Command: {command}")

    if failsafe:
        rospy.logwarn("Failsafe Triggered. Ignoring command.")
        return

    # operation mode
    if command.command_type == SvanCommand.COMMAND_OPERATION_MODE:
        rospy.logdebug(f"Setting operation mode to {command.operation_mode}")
        set_operation_mode(mode=command.operation_mode)

    # linear movement
    elif command.command_type == SvanCommand.COMMAND_MOVEMENT:
        rospy.logdebug(
            f"Setting Velocity: Vel_X: {command.vel_x} Vel_Y: {command.vel_y}"
        )
        set_velocity(vel_x=command.vel_x, vel_y=command.vel_y)

    # vertical height
    elif command.command_type == SvanCommand.COMMAND_HEIGHT:
        rospy.logdebug(f"Setting height to {command.height}")
        set_height(state=command.height)

    # roll
    elif command.command_type == SvanCommand.COMMAND_ROLL:
        rospy.logdebug(f"Setting roll to {command.roll}")
        set_roll(magnitude=command.roll)

    # pitch
    elif command.command_type == SvanCommand.COMMAND_PITCH:
        rospy.logdebug(f"Setting pitch to {command.pitch}")
        set_pitch(magnitude=command.pitch)

    # yaw
    elif command.command_type == SvanCommand.COMMAND_YAW:
        rospy.logdebug(f"Setting yaw to {command.yaw}")
        set_yaw(direction=command.yaw)


rospy.Subscriber("/svan/simple_control", SvanCommand, handle_new_command)
rospy.Subscriber("/svan/joystick_data", Float32MultiArray, override_listener)

if __name__ == "__main__":
    try:
        rospy.loginfo("READY")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
