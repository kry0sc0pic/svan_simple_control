#!/usr/bin/env python3
"""
Pushup + Twirl demo — HTTP Bridge version.

Mirrors examples/pushup_twirl.py but communicates via the HTTP bridge
instead of publishing directly to the ROS topic.

Usage:
    # Simulation (default)
    python3 examples/pushup_twirl_bridge.py

    # Hardware
    python3 examples/pushup_twirl_bridge.py --host 10.42.4.9

Prerequisites:
    pip install requests
    rosrun svan_simple_control bridge.py
"""

import argparse
import sys
import time

import requests


# ---------------------------------------------------------------------------
# Bridge helpers
# ---------------------------------------------------------------------------


def check_bridge(base: str) -> bool:
    """Validate connectivity and warn if the bridge is in mock mode."""
    try:
        response = requests.get(base, timeout=5)
        response.raise_for_status()
        data = response.json()
        if data.get("status") == "warning":
            print(
                "WARNING: Bridge is running in MOCK MODE — commands will NOT reach the robot."
            )
        return True
    except requests.exceptions.ConnectionError:
        print(f"Cannot connect to bridge at {base}")
        print("Make sure the bridge is running:  rosrun svan_simple_control bridge.py")
        return False
    except requests.exceptions.RequestException as exc:
        print(f"Bridge check failed: {exc}")
        return False


def send_command(base: str, endpoint: str, data: dict):
    """POST *data* to *base*/*endpoint* and return the parsed JSON response."""
    url = f"{base}/{endpoint}"
    try:
        response = requests.post(url, json=data, timeout=5)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as exc:
        print(f"Error sending to {url}: {exc}")
        return None


# ---------------------------------------------------------------------------
# Demo sequence
# ---------------------------------------------------------------------------


def run(base: str) -> None:
    print("SVAN Pushup + Twirl demo (HTTP Bridge)")

    # Stop any ongoing operation
    print("Stopping any current operation...")
    result = send_command(base, "mode", {"operation_mode": 1})  # STOP
    print(f"  Response: {result}")
    time.sleep(2)

    # Pushup
    print("PUSHUP")
    result = send_command(base, "mode", {"operation_mode": 3})  # PUSHUP
    print(f"  Response: {result}")
    time.sleep(10)

    # Stop
    print("Stopping...")
    result = send_command(base, "mode", {"operation_mode": 1})  # STOP
    print(f"  Response: {result}")
    time.sleep(2)

    # Twirl
    print("TWIRL")
    result = send_command(base, "mode", {"operation_mode": 2})  # TWIRL
    print(f"  Response: {result}")
    time.sleep(10)

    # Final stop
    print("STOP")
    result = send_command(base, "mode", {"operation_mode": 1})  # STOP
    print(f"  Response: {result}")

    # Print command history
    try:
        history_response = requests.get(f"{base}/history", timeout=5)
        history_response.raise_for_status()
        history_data = history_response.json()
        print("\nCommand history:")
        for i, cmd in enumerate(history_data.get("commands", []), start=1):
            print(f"  {i}. {cmd}")
    except requests.exceptions.RequestException as exc:
        print(f"Error retrieving command history: {exc}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SVAN pushup + twirl demo via HTTP bridge"
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Bridge host (default: 127.0.0.1 for simulation, use 10.42.4.9 for hardware)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8888,
        help="Bridge port (default: 8888)",
    )
    args = parser.parse_args()
    base = f"http://{args.host}:{args.port}"

    if not check_bridge(base):
        sys.exit(1)

    run(base)


if __name__ == "__main__":
    main()
