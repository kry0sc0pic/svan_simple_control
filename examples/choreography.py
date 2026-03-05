#!/usr/bin/env python3
"""
choreography.py
---------------
A scripted multi-step routine that shows off the full breadth of the API:
  1. Walk in
  2. Height wave (up/down)
  3. Attitude display (roll sweep)
  4. Twirl
  5. Pushup sequence
  6. Walk out
  7. Sleep

Introduces a `play_sequence()` helper that accepts a list of
(SvanCommand, hold_seconds) tuples, making it easy to compose new routines
without repeating publish/sleep boilerplate.

Usage:
    rosrun svan_simple_control choreography.py
"""

from svan_simple_control.msg import SvanCommand
import rospy
from typing import List, Tuple

rospy.init_node("choreography_example_node")
c_pub = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=1)
rospy.sleep(1.0)


# ── Helper ────────────────────────────────────────────────────────────────────


def cmd(command_type: int, **kwargs) -> SvanCommand:
    """Shorthand factory for SvanCommand messages."""
    c = SvanCommand()
    c.command_type = command_type
    for k, v in kwargs.items():
        setattr(c, k, v)
    return c


def play_sequence(steps: List[Tuple[SvanCommand, float]]):
    """Publish each command and sleep for its duration."""
    for command, duration in steps:
        c_pub.publish(command)
        rospy.sleep(duration)


# ── Pre-built commands ────────────────────────────────────────────────────────

STOP = cmd(SvanCommand.COMMAND_OPERATION_MODE, operation_mode=SvanCommand.MODE_STOP)
TROT = cmd(SvanCommand.COMMAND_OPERATION_MODE, operation_mode=SvanCommand.MODE_TROT)
TWIRL = cmd(SvanCommand.COMMAND_OPERATION_MODE, operation_mode=SvanCommand.MODE_TWIRL)
PUSHUP = cmd(SvanCommand.COMMAND_OPERATION_MODE, operation_mode=SvanCommand.MODE_PUSHUP)
SLEEP_MODE = cmd(
    SvanCommand.COMMAND_OPERATION_MODE, operation_mode=SvanCommand.MODE_SLEEP
)

FORWARD = cmd(SvanCommand.COMMAND_MOVEMENT, vel_x=0.0, vel_y=0.5)
HALT = cmd(SvanCommand.COMMAND_MOVEMENT, vel_x=0.0, vel_y=0.0)

HEIGHT_UP = cmd(SvanCommand.COMMAND_HEIGHT, height=SvanCommand.HEIGHT_UP)
HEIGHT_DOWN = cmd(SvanCommand.COMMAND_HEIGHT, height=SvanCommand.HEIGHT_DOWN)
HEIGHT_STOP = cmd(SvanCommand.COMMAND_HEIGHT, height=SvanCommand.STOP_HEIGHT)

ROLL_RIGHT = cmd(SvanCommand.COMMAND_ROLL, roll=0.8)
ROLL_LEFT = cmd(SvanCommand.COMMAND_ROLL, roll=-0.8)
ROLL_CENTER = cmd(SvanCommand.COMMAND_ROLL, roll=0.0)

PITCH_FWD = cmd(SvanCommand.COMMAND_PITCH, pitch=0.6)
PITCH_BACK = cmd(SvanCommand.COMMAND_PITCH, pitch=-0.6)
PITCH_CENTER = cmd(SvanCommand.COMMAND_PITCH, pitch=0.0)


# ── Routine ───────────────────────────────────────────────────────────────────

rospy.loginfo("Choreography: starting")

rospy.loginfo("Act 1: walk in")
play_sequence(
    [
        (STOP, 1.5),
        (FORWARD, 3.0),
        (HALT, 0.5),
        (STOP, 1.0),
    ]
)

rospy.loginfo("Act 2: height wave")
play_sequence(
    [
        (HEIGHT_UP, 1.5),
        (HEIGHT_DOWN, 1.5),
        (HEIGHT_UP, 1.5),
        (HEIGHT_STOP, 0.5),
    ]
)

rospy.loginfo("Act 3: attitude display")
play_sequence(
    [
        (ROLL_RIGHT, 1.0),
        (ROLL_LEFT, 1.0),
        (ROLL_CENTER, 0.5),
        (PITCH_FWD, 1.0),
        (PITCH_BACK, 1.0),
        (PITCH_CENTER, 0.5),
    ]
)

rospy.loginfo("Act 4: twirl")
play_sequence(
    [
        (TWIRL, 8.0),
        (STOP, 1.5),
    ]
)

rospy.loginfo("Act 5: pushup sequence")
play_sequence(
    [
        (PUSHUP, 6.0),
        (STOP, 1.5),
        (PUSHUP, 6.0),
        (STOP, 1.5),
    ]
)

rospy.loginfo("Act 6: walk out")
play_sequence(
    [
        (FORWARD, 3.0),
        (HALT, 0.5),
        (STOP, 1.0),
    ]
)

rospy.loginfo("Act 7: sleep")
play_sequence(
    [
        (SLEEP_MODE, 2.0),
    ]
)

rospy.loginfo("Choreography: complete")
