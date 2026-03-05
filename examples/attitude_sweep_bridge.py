#!/usr/bin/env python3
"""
attitude_sweep_bridge.py
------------------------
Same as attitude_sweep.py but uses the HTTP bridge instead of ROS directly.

Requirements:
    pip install requests

Usage:
    python3 examples/attitude_sweep_bridge.py
    python3 examples/attitude_sweep_bridge.py --host 10.42.4.9
"""

import argparse
import sys
import time
import requests

STEPS = 20
STEP_SECS = 0.15
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


def sweep(base: str, endpoint: str, field: str):
    print(f"Attitude sweep: {field}")
    waypoints = [0.0, 1.0, 0.0, -1.0, 0.0]
    prev = 0.0
    for target in waypoints:
        for step in range(STEPS + 1):
            value = prev + (target - prev) * (step / STEPS)
            post(base, endpoint, {field: round(value, 4)})
            time.sleep(STEP_SECS)
        prev = target
    post(base, endpoint, {field: 0.0})
    time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="SVAN attitude sweep via HTTP bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8888, type=int)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    check_bridge(base)

    post(base, "mode", {"operation_mode": MODE_STOP})
    time.sleep(1.0)

    sweep(base, "roll", "roll")
    sweep(base, "pitch", "pitch")

    print("Attitude sweep: complete")
    post(base, "mode", {"operation_mode": MODE_STOP})


if __name__ == "__main__":
    main()
