#!/usr/bin/env python3
"""
keyboard_teleop.py
------------------
Real-time keyboard control of the SVAN over ROS.

Controls:
    w / s       — forward / backward
    a / d       — strafe left / right
    q / e       — yaw left / yaw right
    r / f       — height up / height down
    1           — STOP mode
    2           — TROT mode
    3           — PUSHUP mode
    4           — TWIRL mode
    5           — SLEEP mode
    Space       — halt movement (vel = 0, stays in current mode)
    Esc / Ctrl+C — exit and stop the robot

Requirements:
    pip install pynput

Usage:
    rosrun svan_simple_control keyboard_teleop.py
"""

from svan_simple_control.msg import SvanCommand
import rospy

try:
    from pynput import keyboard as kb
except ImportError:
    print("ERROR: pynput is required. Install with: pip install pynput")
    raise SystemExit(1)

SPEED = 0.5  # normalised speed for movement commands

rospy.init_node("keyboard_teleop_node")
c_pub = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=1)
rospy.sleep(1.0)


# ── Helpers ───────────────────────────────────────────────────────────────────


def pub_mode(mode: int):
    c = SvanCommand()
    c.command_type = SvanCommand.COMMAND_OPERATION_MODE
    c.operation_mode = mode
    c_pub.publish(c)


def pub_movement(vel_x: float, vel_y: float):
    c = SvanCommand()
    c.command_type = SvanCommand.COMMAND_MOVEMENT
    c.vel_x = vel_x
    c.vel_y = vel_y
    c_pub.publish(c)


def pub_yaw(direction: int):
    c = SvanCommand()
    c.command_type = SvanCommand.COMMAND_YAW
    c.yaw = direction
    c_pub.publish(c)


def pub_height(state: int):
    c = SvanCommand()
    c.command_type = SvanCommand.COMMAND_HEIGHT
    c.height = state
    c_pub.publish(c)


# ── Key map ───────────────────────────────────────────────────────────────────

KEY_MAP = {
    "w": lambda: pub_movement(0.0, SPEED),
    "s": lambda: pub_movement(0.0, -SPEED),
    "a": lambda: pub_movement(-SPEED, 0.0),
    "d": lambda: pub_movement(SPEED, 0.0),
    "q": lambda: pub_yaw(SvanCommand.YAW_LEFT),
    "e": lambda: pub_yaw(SvanCommand.YAW_RIGHT),
    "r": lambda: pub_height(SvanCommand.HEIGHT_UP),
    "f": lambda: pub_height(SvanCommand.HEIGHT_DOWN),
    "1": lambda: pub_mode(SvanCommand.MODE_STOP),
    "2": lambda: pub_mode(SvanCommand.MODE_TROT),
    "3": lambda: pub_mode(SvanCommand.MODE_PUSHUP),
    "4": lambda: pub_mode(SvanCommand.MODE_TWIRL),
    "5": lambda: pub_mode(SvanCommand.MODE_SLEEP),
}

RELEASE_MAP = {
    "w": lambda: pub_movement(0.0, 0.0),
    "s": lambda: pub_movement(0.0, 0.0),
    "a": lambda: pub_movement(0.0, 0.0),
    "d": lambda: pub_movement(0.0, 0.0),
    "q": lambda: pub_yaw(SvanCommand.YAW_NONE),
    "e": lambda: pub_yaw(SvanCommand.YAW_NONE),
    "r": lambda: pub_height(SvanCommand.STOP_HEIGHT),
    "f": lambda: pub_height(SvanCommand.STOP_HEIGHT),
}

_running = True


def on_press(key):
    try:
        char = key.char
        if char in KEY_MAP:
            KEY_MAP[char]()
    except AttributeError:
        if key == kb.Key.space:
            pub_movement(0.0, 0.0)


def on_release(key):
    global _running
    if key == kb.Key.esc:
        _running = False
        return False  # stop listener
    try:
        char = key.char
        if char in RELEASE_MAP:
            RELEASE_MAP[char]()
    except AttributeError:
        pass


# ── Main ──────────────────────────────────────────────────────────────────────

print("Keyboard teleop started. Controls:")
print("  w/s/a/d  — move    q/e — yaw    r/f — height")
print("  1-5      — modes   Space — halt   Esc — quit")

pub_mode(SvanCommand.MODE_STOP)

listener = kb.Listener(on_press=on_press, on_release=on_release)
listener.start()

try:
    while _running and not rospy.is_shutdown():
        rospy.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    listener.stop()
    rospy.loginfo("Keyboard teleop: shutting down")
    pub_movement(0.0, 0.0)
    rospy.sleep(0.2)
    pub_mode(SvanCommand.MODE_STOP)
