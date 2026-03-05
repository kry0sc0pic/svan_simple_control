#!/usr/bin/env python3
"""
patrol_bridge.py
----------------
Same as patrol.py but uses the HTTP bridge instead of ROS directly.
Press Ctrl+C to stop cleanly.

Requirements:
    pip install requests

Usage:
    python3 examples/patrol_bridge.py
    python3 examples/patrol_bridge.py --host 10.42.4.9
"""

import argparse
import signal
import sys
import time
import requests

FORWARD_SPEED = 0.5
LEG_SECS = 4.0
TURN_SECS = 3.5

MODE_STOP = 1
YAW_RIGHT = 1
YAW_NONE = 2

_base = None


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


def handle_sigint(sig, frame):
    print("\nPatrol: shutting down — stopping robot")
    if _base:
        post(_base, "movement", {"vel_x": 0.0, "vel_y": 0.0})
        time.sleep(0.3)
        post(_base, "mode", {"operation_mode": MODE_STOP})
    sys.exit(0)


def main():
    global _base
    parser = argparse.ArgumentParser(description="SVAN patrol example via HTTP bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8888, type=int)
    args = parser.parse_args()

    _base = f"http://{args.host}:{args.port}"
    signal.signal(signal.SIGINT, handle_sigint)
    check_bridge(_base)

    print("Patrol: starting — Ctrl+C to stop")
    post(_base, "mode", {"operation_mode": MODE_STOP})
    time.sleep(1.5)

    leg = 0
    while True:
        leg += 1
        print(f"Patrol: leg {leg} — forward")
        post(_base, "movement", {"vel_x": 0.0, "vel_y": FORWARD_SPEED})
        time.sleep(LEG_SECS)

        print(f"Patrol: leg {leg} — halt before turn")
        post(_base, "movement", {"vel_x": 0.0, "vel_y": 0.0})
        time.sleep(0.5)

        print(f"Patrol: leg {leg} — 180° turn")
        post(_base, "yaw", {"yaw": YAW_RIGHT})
        time.sleep(TURN_SECS)
        post(_base, "yaw", {"yaw": YAW_NONE})
        time.sleep(0.5)


if __name__ == "__main__":
    main()
