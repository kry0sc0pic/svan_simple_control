#!/usr/bin/env python3
"""
attitude_sweep.py
-----------------
Sweeps roll and pitch independently through their full range (-1 → 0 → 1 → 0),
demonstrating the body orientation API. The robot stays in place — no locomotion.

Tuning:
    STEPS      — number of increments per sweep (higher = smoother)
    STEP_SECS  — pause between each increment

Usage:
    rosrun svan_simple_control attitude_sweep.py
"""

from svan_simple_control.msg import SvanCommand
import rospy

STEPS = 20
STEP_SECS = 0.15

rospy.init_node("attitude_sweep_example_node")
c_pub = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=1)
rospy.sleep(1.0)

stop_cmd = SvanCommand()
stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
stop_cmd.operation_mode = SvanCommand.MODE_STOP
c_pub.publish(stop_cmd)
rospy.sleep(1.0)


def sweep_roll():
    """Sweep roll: 0 → 1 → 0 → -1 → 0"""
    rospy.loginfo("Attitude sweep: roll")
    waypoints = [0.0, 1.0, 0.0, -1.0, 0.0]
    for target in waypoints:
        # Interpolate current → target over STEPS
        cmd = SvanCommand()
        cmd.command_type = SvanCommand.COMMAND_ROLL
        # Send intermediate steps for smooth motion
        prev = 0.0
        for step in range(STEPS + 1):
            value = prev + (target - prev) * (step / STEPS)
            cmd.roll = round(value, 4)
            c_pub.publish(cmd)
            rospy.sleep(STEP_SECS)
        prev = target

    # Return to neutral
    cmd = SvanCommand()
    cmd.command_type = SvanCommand.COMMAND_ROLL
    cmd.roll = 0.0
    c_pub.publish(cmd)
    rospy.sleep(0.5)


def sweep_pitch():
    """Sweep pitch: 0 → 1 → 0 → -1 → 0"""
    rospy.loginfo("Attitude sweep: pitch")
    waypoints = [0.0, 1.0, 0.0, -1.0, 0.0]
    for target in waypoints:
        cmd = SvanCommand()
        cmd.command_type = SvanCommand.COMMAND_PITCH
        prev = 0.0
        for step in range(STEPS + 1):
            value = prev + (target - prev) * (step / STEPS)
            cmd.pitch = round(value, 4)
            c_pub.publish(cmd)
            rospy.sleep(STEP_SECS)
        prev = target

    cmd = SvanCommand()
    cmd.command_type = SvanCommand.COMMAND_PITCH
    cmd.pitch = 0.0
    c_pub.publish(cmd)
    rospy.sleep(0.5)


sweep_roll()
sweep_pitch()

rospy.loginfo("Attitude sweep: complete")
c_pub.publish(stop_cmd)
