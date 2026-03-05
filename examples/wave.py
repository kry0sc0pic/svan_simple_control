#!/usr/bin/env python3
"""
wave.py
-------
Cycles the robot's height between UP and DOWN repeatedly, creating a
bowing/waving motion. Good first demo — no locomotion required.

Usage:
    rosrun svan_simple_control wave.py
"""

from svan_simple_control.msg import SvanCommand
import rospy

REPS = 5  # number of up/down cycles
HOLD_SECS = 2.0  # seconds to hold each height position

rospy.init_node("wave_example_node")
c_pub = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=1)

# Allow publisher to connect before sending the first command
rospy.sleep(1.0)

# Make sure we start in STOP mode
stop_cmd = SvanCommand()
stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
stop_cmd.operation_mode = SvanCommand.MODE_STOP
c_pub.publish(stop_cmd)
rospy.sleep(1.0)

up_cmd = SvanCommand()
up_cmd.command_type = SvanCommand.COMMAND_HEIGHT
up_cmd.height = SvanCommand.HEIGHT_UP

down_cmd = SvanCommand()
down_cmd.command_type = SvanCommand.COMMAND_HEIGHT
down_cmd.height = SvanCommand.HEIGHT_DOWN

stop_height_cmd = SvanCommand()
stop_height_cmd.command_type = SvanCommand.COMMAND_HEIGHT
stop_height_cmd.height = SvanCommand.STOP_HEIGHT

rospy.loginfo(f"Starting wave — {REPS} cycles")

for i in range(REPS):
    rospy.loginfo(f"Cycle {i + 1}/{REPS}: UP")
    c_pub.publish(up_cmd)
    rospy.sleep(HOLD_SECS)

    rospy.loginfo(f"Cycle {i + 1}/{REPS}: DOWN")
    c_pub.publish(down_cmd)
    rospy.sleep(HOLD_SECS)

rospy.loginfo("Returning to neutral height")
c_pub.publish(stop_height_cmd)
rospy.sleep(1.0)

rospy.loginfo("Done")
c_pub.publish(stop_cmd)
