#!/usr/bin/env python3
"""
gamepad_bridge.py
-----------------
Same as gamepad.py but uses the HTTP bridge instead of ROS directly.
Run this from any laptop on the same network — no ROS installation required.

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

Requirements:
    pip install pygame requests

Usage:
    python3 examples/gamepad_bridge.py
    python3 examples/gamepad_bridge.py --host 10.42.4.9
"""

import argparse
import sys
import time
import requests

try:
    import pygame
except ImportError:
    print("ERROR: pygame is required. Install with: pip install pygame")
    sys.exit(1)

DEADZONE = 0.12
YAW_THRESHOLD = 0.5
SPEED_SCALE = 0.8
LOOP_HZ = 20

MODE_STOP = 1
MODE_TROT = 4
MODE_PUSHUP = 3
MODE_TWIRL = 2
MODE_SLEEP = 5

HEIGHT_UP = 1
HEIGHT_DOWN = 2
STOP_HEIGHT = 3

YAW_LEFT = 0
YAW_RIGHT = 1
YAW_NONE = 2

_base = None


def post(endpoint: str, body: dict):
    url = f"{_base}/{endpoint}"
    try:
        requests.post(url, json=body, timeout=2)
    except requests.exceptions.RequestException as e:
        print(f"WARNING: {endpoint} failed — {e}")


def check_bridge(base: str):
    try:
        r = requests.get(base, timeout=5)
        data = r.json()
        if data.get("status") == "warning":
            print(
                "WARNING: Bridge is running in MOCK MODE — commands won't reach the robot."
            )
    except requests.exceptions.ConnectionError:
        print(f"ERROR: Cannot connect to bridge at {base}")
        print("Make sure the bridge is running: rosrun svan_simple_control bridge.py")
        sys.exit(1)


def clamp(v: float) -> float:
    return max(-1.0, min(v, 1.0))


def apply_deadzone(v: float) -> float:
    return v if abs(v) > DEADZONE else 0.0


BUTTON_MODES = {
    0: MODE_STOP,
    1: MODE_TROT,
    2: MODE_PUSHUP,
    3: MODE_TWIRL,
    7: MODE_SLEEP,
}


def main():
    global _base
    parser = argparse.ArgumentParser(description="SVAN gamepad control via HTTP bridge")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Bridge host (default: 127.0.0.1 for simulation, use 10.42.4.9 for hardware)",
    )
    parser.add_argument("--port", default=8888, type=int)
    args = parser.parse_args()

    _base = f"http://{args.host}:{args.port}"
    check_bridge(_base)

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("ERROR: No gamepad detected. Connect a USB gamepad and retry.")
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Gamepad connected: {joy.get_name()}")

    post("mode", {"operation_mode": MODE_STOP})
    time.sleep(0.5)

    print("Gamepad teleop ready — Press A/Cross to stop, B/Circle to trot")

    prev_yaw = YAW_NONE
    interval = 1.0 / LOOP_HZ

    try:
        while True:
            pygame.event.pump()

            # ── Buttons ───────────────────────────────────────────────────────
            for btn_idx, mode in BUTTON_MODES.items():
                if joy.get_button(btn_idx):
                    print(f"Button {btn_idx}: mode {mode}")
                    post("mode", {"operation_mode": mode})

            if joy.get_button(4):
                post("height", {"height": HEIGHT_DOWN})
            elif joy.get_button(5):
                post("height", {"height": HEIGHT_UP})

            # ── Left stick — movement ─────────────────────────────────────────
            vel_x = clamp(apply_deadzone(joy.get_axis(0)) * SPEED_SCALE)
            vel_y = clamp(-apply_deadzone(joy.get_axis(1)) * SPEED_SCALE)
            post("movement", {"vel_x": vel_x, "vel_y": vel_y})

            # ── Right stick X — yaw ───────────────────────────────────────────
            rx = apply_deadzone(joy.get_axis(3))
            if rx > YAW_THRESHOLD:
                new_yaw = YAW_RIGHT
            elif rx < -YAW_THRESHOLD:
                new_yaw = YAW_LEFT
            else:
                new_yaw = YAW_NONE

            if new_yaw != prev_yaw:
                post("yaw", {"yaw": new_yaw})
                prev_yaw = new_yaw

            time.sleep(interval)

    except KeyboardInterrupt:
        pass
    finally:
        print("Gamepad: shutting down")
        post("movement", {"vel_x": 0.0, "vel_y": 0.0})
        time.sleep(0.2)
        post("mode", {"operation_mode": MODE_STOP})
        pygame.quit()


if __name__ == "__main__":
    main()
