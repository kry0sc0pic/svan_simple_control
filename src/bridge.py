#!/usr/bin/env python3
import rospy
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
from typing import Optional
from svan_simple_control_msgs.msg import SvanCommand

# Initialize FastAPI app
app = FastAPI(title="SVAN Control Bridge",
              description="HTTP API to control SVAN robot using ROS messages")

# Initialize ROS node
try:
    rospy.init_node('svan_bridge_node', anonymous=True, disable_signals=True)
    # Publisher for control commands
    command_publisher = rospy.Publisher('/svan/simple_control', SvanCommand, queue_size=1)
    ros_connected = True
except Exception as e:
    print(f"Warning: Failed to initialize ROS node: {e}")
    print("Running in mock mode - commands will be logged but not sent to ROS")
    ros_connected = False
    
# Command history for verification
command_history = []

# Models for request validation
class OperationModeRequest(BaseModel):
    operation_mode: int = Field(..., description="Operation mode (1=STOP, 2=TWIRL, 3=PUSHUP, 4=TROT, 5=SLEEP)")

class MovementRequest(BaseModel):
    vel_x: float = Field(..., description="Velocity along X axis (-1.0 to 1.0)")
    vel_y: float = Field(..., description="Velocity along Y axis (-1.0 to 1.0)")

class RollRequest(BaseModel):
    roll: float = Field(..., description="Roll angle (-1.0 to 1.0)")

class PitchRequest(BaseModel):
    pitch: float = Field(..., description="Pitch angle (-1.0 to 1.0)")

class YawRequest(BaseModel):
    yaw: int = Field(..., description="Yaw direction (0=LEFT, 1=RIGHT, 2=NONE)")

class HeightRequest(BaseModel):
    height: int = Field(..., description="Height state (1=UP, 2=DOWN)")

@app.get("/")
async def root():
    status = "ok" if ros_connected else "warning"
    message = "SVAN Control Bridge is running. Use /docs for API documentation."
    if not ros_connected:
        message = "SVAN Control Bridge is running in MOCK MODE (ROS not connected). " + message
    return {"status": status, "message": message}

@app.get("/history")
async def get_history():
    return {"status": "ok", "commands": command_history}

@app.post("/mode")
async def set_operation_mode(request: OperationModeRequest):
    cmd_data = {
        "command_type": "OPERATION_MODE",
        "operation_mode": request.operation_mode
    }
    command_history.append(cmd_data)
    
    if ros_connected:
        try:
            cmd = SvanCommand()
            cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
            cmd.operation_mode = request.operation_mode
            command_publisher.publish(cmd)
            return {"status": "ok", "message": f"Set operation mode to {request.operation_mode}"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to set operation mode: {str(e)}")
    else:
        return {"status": "mock", "message": f"Set operation mode to {request.operation_mode} (MOCK MODE)"}

@app.post("/movement")
async def set_movement(request: MovementRequest):
    vel_x = max(-1.0, min(1.0, request.vel_x))  # Constrain values
    vel_y = max(-1.0, min(1.0, request.vel_y))
    
    cmd_data = {
        "command_type": "MOVEMENT",
        "vel_x": vel_x,
        "vel_y": vel_y
    }
    command_history.append(cmd_data)
    
    if ros_connected:
        try:
            cmd = SvanCommand()
            cmd.command_type = SvanCommand.COMMAND_MOVEMENT
            cmd.vel_x = vel_x
            cmd.vel_y = vel_y
            command_publisher.publish(cmd)
            return {"status": "ok", "message": f"Set movement: vel_x={vel_x}, vel_y={vel_y}"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to set movement: {str(e)}")
    else:
        return {"status": "mock", "message": f"Set movement: vel_x={vel_x}, vel_y={vel_y} (MOCK MODE)"}

@app.post("/roll")
async def set_roll(request: RollRequest):
    roll = max(-1.0, min(1.0, request.roll))
    
    cmd_data = {
        "command_type": "ROLL",
        "roll": roll
    }
    command_history.append(cmd_data)
    
    if ros_connected:
        try:
            cmd = SvanCommand()
            cmd.command_type = SvanCommand.COMMAND_ROLL
            cmd.roll = roll
            command_publisher.publish(cmd)
            return {"status": "ok", "message": f"Set roll to {roll}"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to set roll: {str(e)}")
    else:
        return {"status": "mock", "message": f"Set roll to {roll} (MOCK MODE)"}

@app.post("/pitch")
async def set_pitch(request: PitchRequest):
    pitch = max(-1.0, min(1.0, request.pitch))
    
    cmd_data = {
        "command_type": "PITCH",
        "pitch": pitch
    }
    command_history.append(cmd_data)
    
    if ros_connected:
        try:
            cmd = SvanCommand()
            cmd.command_type = SvanCommand.COMMAND_PITCH
            cmd.pitch = pitch
            command_publisher.publish(cmd)
            return {"status": "ok", "message": f"Set pitch to {pitch}"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to set pitch: {str(e)}")
    else:
        return {"status": "mock", "message": f"Set pitch to {pitch} (MOCK MODE)"}

@app.post("/yaw")
async def set_yaw(request: YawRequest):
    cmd_data = {
        "command_type": "YAW",
        "yaw": request.yaw
    }
    command_history.append(cmd_data)
    
    if ros_connected:
        try:
            cmd = SvanCommand()
            cmd.command_type = SvanCommand.COMMAND_YAW
            cmd.yaw = request.yaw
            command_publisher.publish(cmd)
            return {"status": "ok", "message": f"Set yaw to {request.yaw}"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to set yaw: {str(e)}")
    else:
        return {"status": "mock", "message": f"Set yaw to {request.yaw} (MOCK MODE)"}

@app.post("/height")
async def set_height(request: HeightRequest):
    cmd_data = {
        "command_type": "HEIGHT",
        "height": request.height
    }
    command_history.append(cmd_data)
    
    if ros_connected:
        try:
            cmd = SvanCommand()
            cmd.command_type = SvanCommand.COMMAND_HEIGHT
            cmd.height = request.height
            command_publisher.publish(cmd)
            return {"status": "ok", "message": f"Set height to {request.height}"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to set height: {str(e)}")
    else:
        return {"status": "mock", "message": f"Set height to {request.height} (MOCK MODE)"}

if __name__ == "__main__":
    print("Starting SVAN Control Bridge")
    print(f"Access the API at http://0.0.0.0:8888")
    print(f"Access the docs at http://0.0.0.0:8888/docs")
    print(f"View command history at http://0.0.0.0:888/history")
    if not ros_connected:
        print("WARNING: Running in MOCK MODE - commands will not be sent to ROS")
    uvicorn.run(app, host="0.0.0.0", port=8888) 