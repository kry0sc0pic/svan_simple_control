#!/usr/bin/env python3
"""
gamepad.py
----------
USB gamepad control of the SVAN over ROS using pygame.

Axis mapping (standard gamepad layout):
    Left stick X  (axis 0) — strafe  (vel_x)
    Left stick Y  (axis 1) — forward/back (vel_y, inverted)
    Right stick X (axis 3) — yaw left/right (threshold-based)

Button mapping:
    A / Cross    (btn 0) — STOP mode
    B / Circle   (btn 1) — TROT mode
    X / Square   (btn 2) — PUSHUP mode
    Y / Triangle (btn 3) — TWIRL mode
    LB / L1      (btn 4) — height DOWN
    RB / R1      (btn 5) — height UP
    Start        (btn 7) — SLEEP mode

Tuning:
    DEADZONE       — stick deadzone (ignore small drift)
    YAW_THRESHOLD  — right-stick X threshold to trigger yaw
    SPEED_SCALE    — multiplier applied to stick values (≤ 1.0)

Requirements:
    pip install pygame

Usage:
    rosrun svan_simple_control gamepad.py
"""

from svan_simple_control.msg import SvanCommand
import rospy

try:
    import pygame
except ImportError:
    print("ERROR: pygame is required. Install with: pip install pygame")
    raise SystemExit(1)

DEADZONE = 0.12
YAW_THRESHOLD = 0.5
SPEED_SCALE = 0.8
PUBLISH_HZ = 20  # how often to publish axis state


def clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(v, hi))


def apply_deadzone(v: float) -> float:
    return v if abs(v) > DEADZONE else 0.0


rospy.init_node("gamepad_node")
c_pub = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=1)
rate = rospy.Rate(PUBLISH_HZ)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    rospy.logerr("No gamepad detected. Connect a USB gamepad and retry.")
    raise SystemExit(1)

joy = pygame.joystick.Joystick(0)
joy.init()
rospy.loginfo(f"Gamepad connected: {joy.get_name()}")

# Send initial stop
stop_cmd = SvanCommand()
stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
stop_cmd.operation_mode = SvanCommand.MODE_STOP
c_pub.publish(stop_cmd)
rospy.sleep(0.5)

BUTTON_MODES = {
    0: SvanCommand.MODE_STOP,
    1: SvanCommand.MODE_TROT,
    2: SvanCommand.MODE_PUSHUP,
    3: SvanCommand.MODE_TWIRL,
    7: SvanCommand.MODE_SLEEP,
}

prev_yaw = SvanCommand.YAW_NONE


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


rospy.loginfo("Gamepad teleop ready — Press A/Cross to stop, B/Circle to trot")

try:
    while not rospy.is_shutdown():
        pygame.event.pump()

        # ── Buttons ───────────────────────────────────────────────────────────
        for btn_idx, mode in BUTTON_MODES.items():
            if joy.get_button(btn_idx):
                rospy.loginfo(f"Button {btn_idx}: mode {mode}")
                pub_mode(mode)

        if joy.get_button(4):  # LB — height down
            pub_height(SvanCommand.HEIGHT_DOWN)
        elif joy.get_button(5):  # RB — height up
            pub_height(SvanCommand.HEIGHT_UP)

        # ── Left stick — movement ─────────────────────────────────────────────
        raw_x = apply_deadzone(joy.get_axis(0))
        raw_y = -apply_deadzone(joy.get_axis(1))  # invert: push forward = positive
        vel_x = clamp(raw_x * SPEED_SCALE)
        vel_y = clamp(raw_y * SPEED_SCALE)
        pub_movement(vel_x, vel_y)

        # ── Right stick X — yaw ───────────────────────────────────────────────
        global prev_yaw
        rx = apply_deadzone(joy.get_axis(3))
        if rx > YAW_THRESHOLD:
            new_yaw = SvanCommand.YAW_RIGHT
        elif rx < -YAW_THRESHOLD:
            new_yaw = SvanCommand.YAW_LEFT
        else:
            new_yaw = SvanCommand.YAW_NONE

        if new_yaw != prev_yaw:
            pub_yaw(new_yaw)
            # A tiny forward nudge is required for yaw to take effect.
            nudge = 0.0 if new_yaw == SvanCommand.YAW_NONE else 0.0001
            pub_movement(vel_x, vel_y if vel_y != 0.0 else nudge)
            prev_yaw = new_yaw

        rate.sleep()

except KeyboardInterrupt:
    pass
finally:
    rospy.loginfo("Gamepad: shutting down")
    pub_movement(0.0, 0.0)
    rospy.sleep(0.2)
    pub_mode(SvanCommand.MODE_STOP)
    pygame.quit()
