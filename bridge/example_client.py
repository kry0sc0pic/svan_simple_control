#!/usr/bin/env python3
import requests
import time
import sys

# Base URL for the API
BASE_URL = "http://127.0.0.1:8888"

def send_command(endpoint, data):
    """Send a command to the SVAN control bridge"""
    url = f"{BASE_URL}/{endpoint}"
    try:
        response = requests.post(url, json=data)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return None

def main():
    print("SVAN Control HTTP Bridge - Example Client")
    print("Make sure the bridge is running before proceeding.")
    
    # Check if bridge is running
    try:
        response = requests.get(BASE_URL)
        if response.status_code != 200:
            print("Bridge is not responding correctly. Is it running?")
            sys.exit(1)
    except requests.exceptions.ConnectionError:
        print("Cannot connect to bridge. Make sure it's running at", BASE_URL)
        sys.exit(1)
    
    print("\nDemonstrating a simple sequence of commands...")
    
    # Stop any current operation
    print("1. Stopping any current operation")
    send_command("mode", {"operation_mode": 1})  # STOP mode
    time.sleep(2)
    
    # Switch to trot mode
    print("2. Switching to TROT mode")
    send_command("mode", {"operation_mode": 4})  # TROT mode
    time.sleep(2)
    
    # Move forward
    print("3. Moving forward at half speed")
    send_command("movement", {"vel_x": 0.0, "vel_y": 0.5})
    time.sleep(5)
    
    # Turn right
    print("4. Turning right while moving")
    send_command("yaw", {"yaw": 1})  # YAW_RIGHT
    time.sleep(5)
    
    # Stop turning
    print("5. Stop turning")
    send_command("yaw", {"yaw": 2})  # YAW_NONE
    time.sleep(2)
    
    # Stop movement
    print("6. Stopping movement")
    send_command("movement", {"vel_x": 0.0, "vel_y": 0.0})
    time.sleep(1)
    
    # Do a pushup
    print("7. Doing a pushup")
    send_command("mode", {"operation_mode": 3})  # PUSHUP mode
    time.sleep(10)
    
    # Back to stop mode
    print("8. Stopping")
    send_command("mode", {"operation_mode": 1})  # STOP mode
    
    print("\nDemo completed!")

if __name__ == "__main__":
    main() 