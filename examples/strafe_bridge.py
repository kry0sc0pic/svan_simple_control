#!/usr/bin/env python3
"""
strafe_bridge.py
----------------
Same as strafe.py but uses the HTTP bridge instead of ROS directly.

Requirements:
    pip install requests

Usage:
    python3 examples/strafe_bridge.py
    python3 examples/strafe_bridge.py --host 10.42.4.9
"""

import argparse
import sys
import time
import requests

SPEED = 0.5
LEG_SECS = 3.0
MODE_STOP = 1


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


def move(base: str, vel_x: float, vel_y: float, label: str):
    print(f"Strafe: {label}")
    post(base, "movement", {"vel_x": vel_x, "vel_y": vel_y})
    time.sleep(LEG_SECS)
    post(base, "movement", {"vel_x": 0.0, "vel_y": 0.0})
    time.sleep(0.4)


def main():
    parser = argparse.ArgumentParser(description="SVAN strafe example via HTTP bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8888, type=int)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    check_bridge(base)

    print("Strafe: starting diamond pattern")
    post(base, "mode", {"operation_mode": MODE_STOP})
    time.sleep(1.5)

    move(base, 0.0, SPEED, "forward")
    move(base, SPEED, 0.0, "strafe right")
    move(base, 0.0, -SPEED, "backward")
    move(base, -SPEED, 0.0, "strafe left")

    print("Strafe: complete — stopping")
    post(base, "mode", {"operation_mode": MODE_STOP})


if __name__ == "__main__":
    main()
