#!/usr/bin/env python3
import requests
import time
import sys

# Base URL for the SVAN Bridge API
BASE_URL = "http://127.0.0.1:8888"

def send_command(endpoint, data):
    """Send a command to the SVAN control bridge"""
    url = f"{BASE_URL}/{endpoint}"
    try:
        response = requests.post(url, json=data)
        response.raise_for_status()  # Raise an exception for HTTP errors
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return None

def check_bridge_status():
    """Check if the bridge is running"""
    try:
        response = requests.get(BASE_URL)
        if response.status_code != 200:
            print("Bridge is not responding correctly. Is it running?")
            return False
        
        status_data = response.json()
        if status_data.get("status") == "warning":
            print("WARNING: Bridge is running in MOCK MODE. Commands won't be sent to the robot.")
        
        return True
    except requests.exceptions.ConnectionError:
        print(f"Cannot connect to bridge at {BASE_URL}")
        print("Please make sure the bridge is running:")
        print("  1. Activate the virtual environment: source bridge_venv/bin/activate")
        print("  2. Start the bridge: python3 bridge/bridge.py")
        return False

def main():
    print("SVAN HTTP Pushup Example - Using Bridge API")
    
    # Check if bridge is running
    if not check_bridge_status():
        sys.exit(1)
    
    # First stop any current operation
    print("Stopping any current operation")
    result = send_command("mode", {"operation_mode": 1})  # STOP mode
    print(f"Response: {result}")
    time.sleep(2)
    
    # Enter pushup mode
    print("PUSHUP")
    result = send_command("mode", {"operation_mode": 3})  # PUSHUP mode
    print(f"Response: {result}")
    time.sleep(10)
    
    # Stop
    print("Stopping")
    result = send_command("mode", {"operation_mode": 1})  # STOP mode
    print(f"Response: {result}")
    time.sleep(2)
    
    # Enter twirl mode
    print("TWIRL")
    result = send_command("mode", {"operation_mode": 2})  # TWIRL mode
    print(f"Response: {result}")
    time.sleep(10)
    
    # Final stop
    print("STOP")
    result = send_command("mode", {"operation_mode": 1})  # STOP mode
    print(f"Response: {result}")
    
    # Check command history
    try:
        history_response = requests.get(f"{BASE_URL}/history")
        history_response.raise_for_status()
        history_data = history_response.json()
        
        print("\nCommand History:")
        for i, cmd in enumerate(history_data.get("commands", [])):
            print(f"  {i+1}. {cmd}")
    except requests.exceptions.RequestException as e:
        print(f"Error retrieving command history: {e}")

if __name__ == "__main__":
    main() 