#!/usr/bin/env python3
"""
patrol.py
---------
Runs an indefinite back-and-forth patrol: walk forward for LEG_SECS,
stop briefly, turn 180°, repeat. Press Ctrl+C to stop cleanly.

Useful for continuous lab demos and testing sustained locomotion.

Tuning:
    FORWARD_SPEED  — trot speed during patrol legs
    LEG_SECS       — duration of each forward leg
    TURN_SECS      — duration of the 180° turn (tune per surface)

Usage:
    rosrun svan_simple_control patrol.py
"""

import signal
import rospy
from svan_simple_control.msg import SvanCommand

FORWARD_SPEED = 0.5
LEG_SECS = 4.0
TURN_SECS = 3.5  # tune for ~180° on your surface

rospy.init_node("patrol_example_node")
c_pub = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=1)
rospy.sleep(1.0)

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

# A tiny forward nudge is required for yaw to take effect.
yaw_nudge_cmd = SvanCommand()
yaw_nudge_cmd.command_type = SvanCommand.COMMAND_MOVEMENT
yaw_nudge_cmd.vel_x = 0.0
yaw_nudge_cmd.vel_y = 0.0001


def shutdown():
    rospy.loginfo("Patrol: shutting down — stopping robot")
    c_pub.publish(halt_cmd)
    rospy.sleep(0.3)
    c_pub.publish(stop_cmd)


rospy.on_shutdown(shutdown)

rospy.loginfo("Patrol: starting — Ctrl+C to stop")
c_pub.publish(stop_cmd)
rospy.sleep(1.5)

leg = 0
while not rospy.is_shutdown():
    leg += 1
    rospy.loginfo(f"Patrol: leg {leg} — forward")
    c_pub.publish(forward_cmd)
    rospy.sleep(LEG_SECS)

    rospy.loginfo(f"Patrol: leg {leg} — halt before turn")
    c_pub.publish(halt_cmd)
    rospy.sleep(0.5)

    rospy.loginfo(f"Patrol: leg {leg} — 180° turn")
    c_pub.publish(yaw_right_cmd)
    c_pub.publish(yaw_nudge_cmd)
    rospy.sleep(TURN_SECS)
    c_pub.publish(yaw_none_cmd)
    rospy.sleep(0.5)
