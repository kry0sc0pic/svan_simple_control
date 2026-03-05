#!/usr/bin/env python3
"""
choreography_bridge.py
----------------------
Same as choreography.py but uses the HTTP bridge instead of ROS directly.

Introduces a `play_sequence()` helper that accepts a list of
(endpoint, body, hold_seconds) tuples, making it easy to compose routines
without repeating requests/sleep boilerplate.

Requirements:
    pip install requests

Usage:
    python3 examples/choreography_bridge.py
    python3 examples/choreography_bridge.py --host 10.42.4.9
"""

import argparse
import sys
import time
import requests
from typing import List, Tuple

# ── Bridge constants (mirror SvanCommand values) ──────────────────────────────
MODE_STOP = 1
MODE_TWIRL = 2
MODE_PUSHUP = 3
MODE_TROT = 4
MODE_SLEEP = 5

HEIGHT_UP = 1
HEIGHT_DOWN = 2
STOP_HEIGHT = 3

YAW_LEFT = 0
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


def play_sequence(base: str, steps: List[Tuple[str, dict, float]]):
    """Post each command and sleep for its duration."""
    for endpoint, body, duration in steps:
        post(base, endpoint, body)
        time.sleep(duration)


def main():
    parser = argparse.ArgumentParser(description="SVAN choreography via HTTP bridge")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Bridge host (default: 127.0.0.1 for simulation, use 10.42.4.9 for hardware)",
    )
    parser.add_argument("--port", default=8888, type=int)
    args = parser.parse_args()

    base = f"http://{args.host}:{args.port}"
    check_bridge(base)

    print("Choreography: starting")

    print("Act 1: walk in")
    play_sequence(
        base,
        [
            ("mode", {"operation_mode": MODE_STOP}, 1.5),
            ("movement", {"vel_x": 0.0, "vel_y": 0.5}, 3.0),
            ("movement", {"vel_x": 0.0, "vel_y": 0.0}, 0.5),
            ("mode", {"operation_mode": MODE_STOP}, 1.0),
        ],
    )

    print("Act 2: height wave")
    play_sequence(
        base,
        [
            ("height", {"height": HEIGHT_UP}, 1.5),
            ("height", {"height": HEIGHT_DOWN}, 1.5),
            ("height", {"height": HEIGHT_UP}, 1.5),
            ("height", {"height": STOP_HEIGHT}, 0.5),
        ],
    )

    print("Act 3: attitude display")
    play_sequence(
        base,
        [
            ("roll", {"roll": 0.8}, 1.0),
            ("roll", {"roll": -0.8}, 1.0),
            ("roll", {"roll": 0.0}, 0.5),
            ("pitch", {"pitch": 0.6}, 1.0),
            ("pitch", {"pitch": -0.6}, 1.0),
            ("pitch", {"pitch": 0.0}, 0.5),
        ],
    )

    print("Act 4: twirl")
    play_sequence(
        base,
        [
            ("mode", {"operation_mode": MODE_TWIRL}, 8.0),
            ("mode", {"operation_mode": MODE_STOP}, 1.5),
        ],
    )

    print("Act 5: pushup sequence")
    play_sequence(
        base,
        [
            ("mode", {"operation_mode": MODE_PUSHUP}, 6.0),
            ("mode", {"operation_mode": MODE_STOP}, 1.5),
            ("mode", {"operation_mode": MODE_PUSHUP}, 6.0),
            ("mode", {"operation_mode": MODE_STOP}, 1.5),
        ],
    )

    print("Act 6: walk out")
    play_sequence(
        base,
        [
            ("movement", {"vel_x": 0.0, "vel_y": 0.5}, 3.0),
            ("movement", {"vel_x": 0.0, "vel_y": 0.0}, 0.5),
            ("mode", {"operation_mode": MODE_STOP}, 1.0),
        ],
    )

    print("Act 7: sleep")
    play_sequence(
        base,
        [
            ("mode", {"operation_mode": MODE_SLEEP}, 2.0),
        ],
    )

    print("Choreography: complete")


if __name__ == "__main__":
    main()
