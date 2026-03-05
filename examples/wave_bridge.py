#!/usr/bin/env python3
"""
wave_bridge.py
--------------
Same as wave.py but uses the HTTP bridge instead of ROS directly.
Run this from any machine on the same network as the robot.

Requirements:
    pip install requests

Usage:
    python3 examples/wave_bridge.py
    python3 examples/wave_bridge.py --host 10.42.4.9  # custom host
"""

import argparse
import sys
import time
import requests

REPS = 5
HOLD_SECS = 2.0

# HEIGHT values (mirror SvanCommand constants)
HEIGHT_UP = 1
HEIGHT_DOWN = 2
STOP_HEIGHT = 3

# MODE values
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


def main():
    parser = argparse.ArgumentParser(description="SVAN wave example via HTTP bridge")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Bridge host (default: 127.0.0.1 for simulation, use 10.42.4.9 for hardware)",
    )
    parser.add_argument(
        "--port", default=8888, type=int, help="Bridge port (default: 8888)"
    )
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    check_bridge(base)

    print("Stopping before start")
    post(base, "mode", {"operation_mode": MODE_STOP})
    time.sleep(1.0)

    print(f"Starting wave — {REPS} cycles")
    for i in range(REPS):
        print(f"Cycle {i + 1}/{REPS}: UP")
        post(base, "height", {"height": HEIGHT_UP})
        time.sleep(HOLD_SECS)

        print(f"Cycle {i + 1}/{REPS}: DOWN")
        post(base, "height", {"height": HEIGHT_DOWN})
        time.sleep(HOLD_SECS)

    print("Returning to neutral height")
    post(base, "height", {"height": STOP_HEIGHT})
    time.sleep(1.0)

    print("Done — stopping")
    post(base, "mode", {"operation_mode": MODE_STOP})


if __name__ == "__main__":
    main()
