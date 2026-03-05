#!/usr/bin/env python3
"""
strafe.py
---------
Demonstrates lateral (vel_x) movement by walking a diamond pattern:
forward → strafe right → backward → strafe left → stop.

This is the primary example for vel_x, which no other basic example
exercises directly.

Tuning:
    SPEED      — normalised speed for all legs of the diamond
    LEG_SECS   — duration of each leg

Usage:
    rosrun svan_simple_control strafe.py
"""

from svan_simple_control.msg import SvanCommand
import rospy

SPEED = 0.5
LEG_SECS = 3.0

rospy.init_node("strafe_example_node")
c_pub = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=1)
rospy.sleep(1.0)

stop_cmd = SvanCommand()
stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
stop_cmd.operation_mode = SvanCommand.MODE_STOP

halt_cmd = SvanCommand()
halt_cmd.command_type = SvanCommand.COMMAND_MOVEMENT
halt_cmd.vel_x = 0.0
halt_cmd.vel_y = 0.0


def move(vel_x: float, vel_y: float, label: str):
    rospy.loginfo(f"Strafe: {label}")
    cmd = SvanCommand()
    cmd.command_type = SvanCommand.COMMAND_MOVEMENT
    cmd.vel_x = vel_x
    cmd.vel_y = vel_y
    c_pub.publish(cmd)
    rospy.sleep(LEG_SECS)
    c_pub.publish(halt_cmd)
    rospy.sleep(0.4)


rospy.loginfo("Strafe: starting diamond pattern")
c_pub.publish(stop_cmd)
rospy.sleep(1.5)

move(0.0, SPEED, "forward")
move(SPEED, 0.0, "strafe right")
move(0.0, -SPEED, "backward")
move(-SPEED, 0.0, "strafe left")

rospy.loginfo("Strafe: complete — stopping")
c_pub.publish(stop_cmd)
