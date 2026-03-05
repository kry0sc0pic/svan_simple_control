#!/usr/bin/env python3
"""
square.py
---------
Walks the robot in an approximate square: forward, turn 90° right, repeat
four times, then stop. Demonstrates combining COMMAND_MOVEMENT and
COMMAND_YAW in a structured sequence.

Tuning:
    SIDE_SECS  — how long to walk each side (longer = bigger square)
    TURN_SECS  — how long to yaw to achieve ~90° (tune per surface/speed)

Usage:
    rosrun svan_simple_control square.py
"""

from svan_simple_control.msg import SvanCommand
import rospy

FORWARD_SPEED = 0.5  # vel_y, -1.0 to 1.0
SIDE_SECS = 3.0  # seconds of forward motion per side
TURN_SECS = 1.8  # seconds of yawing to approximate 90°

rospy.init_node("square_example_node")
c_pub = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=1)
rospy.sleep(1.0)

# ── Pre-built commands ────────────────────────────────────────────────────────
stop_cmd = SvanCommand()
stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
stop_cmd.operation_mode = SvanCommand.MODE_STOP

forward_cmd = SvanCommand()
forward_cmd.command_type = SvanCommand.COMMAND_MOVEMENT
forward_cmd.vel_x = 0.0
forward_cmd.vel_y = FORWARD_SPEED

halt_cmd = SvanCommand()
halt_cmd.command_type = SvanCommand.COMMAND_MOVEMENT
halt_cmd.vel_x = 0.0
halt_cmd.vel_y = 0.0

yaw_right_cmd = SvanCommand()
yaw_right_cmd.command_type = SvanCommand.COMMAND_YAW
yaw_right_cmd.yaw = SvanCommand.YAW_RIGHT

yaw_none_cmd = SvanCommand()
yaw_none_cmd.command_type = SvanCommand.COMMAND_YAW
yaw_none_cmd.yaw = SvanCommand.YAW_NONE

# ── Sequence ──────────────────────────────────────────────────────────────────
rospy.loginfo("Square: starting")
c_pub.publish(stop_cmd)
rospy.sleep(1.5)

for side in range(1, 5):
    rospy.loginfo(f"Square: side {side}/4 — forward")
    c_pub.publish(forward_cmd)
    rospy.sleep(SIDE_SECS)

    rospy.loginfo(f"Square: side {side}/4 — halt")
    c_pub.publish(halt_cmd)
    rospy.sleep(0.5)

    rospy.loginfo(f"Square: side {side}/4 — turn right 90°")
    c_pub.publish(yaw_right_cmd)
    rospy.sleep(TURN_SECS)

    c_pub.publish(yaw_none_cmd)
    rospy.sleep(0.5)

rospy.loginfo("Square: complete — stopping")
c_pub.publish(stop_cmd)
