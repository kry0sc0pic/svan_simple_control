#!/usr/bin/env python3
"""
square_bridge.py
----------------
Same as square.py but uses the HTTP bridge instead of ROS directly.

Requirements:
    pip install requests

Usage:
    python3 examples/square_bridge.py
    python3 examples/square_bridge.py --host 10.42.4.9
"""

import argparse
import sys
import time
import requests

FORWARD_SPEED = 0.5
SIDE_SECS = 3.0
TURN_SECS = 1.8

MODE_STOP = 1
YAW_RIGHT = 1
YAW_NONE = 2


def post(base: str, endpoint: str, body: dict) -> dict:
    url = f"{base}/{endpoint}"
    try:
        r = requests.post(url, json=body, timeout=5)
        r.raise_for_status()
        return r.json()
    except requests.exceptions.RequestException as e:
        print(f"ERROR [{endpoint}]: {e}")
        sys.exit(1)


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


def main():
    parser = argparse.ArgumentParser(description="SVAN square example via HTTP bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8888, type=int)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    check_bridge(base)

    print("Square: starting")
    post(base, "mode", {"operation_mode": MODE_STOP})
    time.sleep(1.5)

    for side in range(1, 5):
        print(f"Square: side {side}/4 — forward")
        post(base, "movement", {"vel_x": 0.0, "vel_y": FORWARD_SPEED})
        time.sleep(SIDE_SECS)

        print(f"Square: side {side}/4 — halt")
        post(base, "movement", {"vel_x": 0.0, "vel_y": 0.0})
        time.sleep(0.5)

        print(f"Square: side {side}/4 — turn right 90°")
        post(base, "yaw", {"yaw": YAW_RIGHT})
        time.sleep(TURN_SECS)

        post(base, "yaw", {"yaw": YAW_NONE})
        time.sleep(0.5)

    print("Square: complete — stopping")
    post(base, "mode", {"operation_mode": MODE_STOP})


if __name__ == "__main__":
    main()
