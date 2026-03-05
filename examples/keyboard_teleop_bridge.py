#!/usr/bin/env python3
"""
keyboard_teleop_bridge.py
-------------------------
Same as keyboard_teleop.py but uses the HTTP bridge instead of ROS directly.
Run this from any laptop on the same network — no ROS installation required.

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
    Space       — halt movement
    Esc / Ctrl+C — exit and stop the robot

Requirements:
    pip install pynput requests

Usage:
    python3 examples/keyboard_teleop_bridge.py
    python3 examples/keyboard_teleop_bridge.py --host 10.42.4.9
"""

import argparse
import sys
import time
import requests

try:
    from pynput import keyboard as kb
except ImportError:
    print("ERROR: pynput is required. Install with: pip install pynput")
    sys.exit(1)

SPEED = 0.5

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


KEY_MAP = {
    "w": lambda: post("movement", {"vel_x": 0.0, "vel_y": SPEED}),
    "s": lambda: post("movement", {"vel_x": 0.0, "vel_y": -SPEED}),
    "a": lambda: post("movement", {"vel_x": -SPEED, "vel_y": 0.0}),
    "d": lambda: post("movement", {"vel_x": SPEED, "vel_y": 0.0}),
    "q": lambda: post("yaw", {"yaw": YAW_LEFT}),
    "e": lambda: post("yaw", {"yaw": YAW_RIGHT}),
    "r": lambda: post("height", {"height": HEIGHT_UP}),
    "f": lambda: post("height", {"height": HEIGHT_DOWN}),
    "1": lambda: post("mode", {"operation_mode": MODE_STOP}),
    "2": lambda: post("mode", {"operation_mode": MODE_TROT}),
    "3": lambda: post("mode", {"operation_mode": MODE_PUSHUP}),
    "4": lambda: post("mode", {"operation_mode": MODE_TWIRL}),
    "5": lambda: post("mode", {"operation_mode": MODE_SLEEP}),
}

RELEASE_MAP = {
    "w": lambda: post("movement", {"vel_x": 0.0, "vel_y": 0.0}),
    "s": lambda: post("movement", {"vel_x": 0.0, "vel_y": 0.0}),
    "a": lambda: post("movement", {"vel_x": 0.0, "vel_y": 0.0}),
    "d": lambda: post("movement", {"vel_x": 0.0, "vel_y": 0.0}),
    "q": lambda: post("yaw", {"yaw": YAW_NONE}),
    "e": lambda: post("yaw", {"yaw": YAW_NONE}),
    "r": lambda: post("height", {"height": STOP_HEIGHT}),
    "f": lambda: post("height", {"height": STOP_HEIGHT}),
}

_running = True


def on_press(key):
    try:
        char = key.char
        if char in KEY_MAP:
            KEY_MAP[char]()
    except AttributeError:
        if key == kb.Key.space:
            post("movement", {"vel_x": 0.0, "vel_y": 0.0})


def on_release(key):
    global _running
    if key == kb.Key.esc:
        _running = False
        return False
    try:
        char = key.char
        if char in RELEASE_MAP:
            RELEASE_MAP[char]()
    except AttributeError:
        pass


def main():
    global _base
    parser = argparse.ArgumentParser(description="SVAN keyboard teleop via HTTP bridge")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Bridge host (default: 127.0.0.1 for simulation, use 10.42.4.9 for hardware)",
    )
    parser.add_argument("--port", default=8888, type=int)
    args = parser.parse_args()

    _base = f"http://{args.host}:{args.port}"
    check_bridge(_base)

    print("Keyboard teleop started. Controls:")
    print("  w/s/a/d  — move    q/e — yaw    r/f — height")
    print("  1-5      — modes   Space — halt   Esc — quit")

    post("mode", {"operation_mode": MODE_STOP})

    listener = kb.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while _running:
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        print("Keyboard teleop: shutting down")
        post("movement", {"vel_x": 0.0, "vel_y": 0.0})
        time.sleep(0.2)
        post("mode", {"operation_mode": MODE_STOP})


if __name__ == "__main__":
    main()
