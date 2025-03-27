#!/usr/bin/env python3
import os
import sys
from svan_simple_control_msgs.msg import SvanCommand
from std_msgs.msg import Float32MultiArray
import rospy


#TODO: control mixing

rospy.init_node('svan_simple_control_node')


key_pub = rospy.Publisher('/svan/io_interface', Float32MultiArray, queue_size=1)

current_operation_mode = -1

# Utility functions
def constrain_value(value,minumum: float = 0.0,maximum: float = 1.0):
    return max(minumum, min(value, maximum))
# for later
def scale_height(height: float):
    constrained_height = max(0.0, min(height, 1.0))
    rescaled_height = 1.5 + (2 * constrained_height)
    return rescaled_height
# Core

def set_operation_mode(mode: int):
    global current_operation_mode
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    if mode == SvanCommand.TROT:
        rospy.loginfo("Trot Mode")
        key_data.data[0] = 4
        current_operation_mode = SvanCommand.TROT
        key_pub.publish(key_data)
    elif mode == SvanCommand.PUSHUP:
        rospy.loginfo("Pushup Mode")
        current_operation_mode = SvanCommand.PUSHUP
        key_data.data[0] = 3
        key_pub.publish(key_data)
    elif mode == SvanCommand.TWIRL:
        rospy.loginfo("Twirl Mode")
        print("Setting operation mode to twirl")
        current_operation_mode = SvanCommand.TWIRL
        key_data.data[0] = 2
        key_pub.publish(key_data)
    elif mode == SvanCommand.STOP:
        rospy.loginfo("Stop Mode")
        print("Setting operation mode to stop")
        current_operation_mode = SvanCommand.STOP
        key_data.data[0] = 1
        key_pub.publish(key_data)
    elif mode == SvanCommand.SLEEP:
        rospy.loginfo("Sleep Mode")
        print("Setting operation mode to sleep")
        current_operation_mode = SvanCommand.SLEEP
        key_data.data[0] = 6
        key_pub.publish(key_data)

def set_movement(direction: int, velocity: float):
    global current_operation_mode
    if current_operation_mode != SvanCommand.TROT:
        rospy.logwarn("Movement commands are only available in trot mode")
        return
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    key_data.data[0] = 4
    constrained_velocity = constrain_value(velocity)
    if direction == SvanCommand.DIRECTION_FORWARD:
        rospy.loginfo(f"Moving forward with velocity {constrained_velocity}", )
        key_data.data[2] = constrained_velocity
    elif direction == SvanCommand.DIRECTION_BACKWARD:
        rospy.loginfo(f"Moving backward with velocity {constrained_velocity}", )
        key_data.data[2] = -1 * constrained_velocity
    elif direction == SvanCommand.DIRECTION_LEFT:
        rospy.loginfo(f"Moving left with velocity {constrained_velocity}", )
        key_data.data[1] = -1 * constrained_velocity
    elif direction == SvanCommand.DIRECTION_RIGHT:
        rospy.loginfo(f"Moving right with velocity {constrained_velocity}", )
        key_data.data[1] = constrained_velocity
    elif direction == SvanCommand.DIRECTION_NONE:
        rospy.loginfo("Stopping movement")
        key_data.data[1:] = [0] * 8
    key_pub.publish(key_data)

def set_height(cmd: int):
    if current_operation_mode != SvanCommand.TROT:
        rospy.logwarn("Height commands are only available in trot mode")
        return
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    key_data.data[0] = 4
    if cmd == SvanCommand.HEIGHT_UP:
        rospy.loginfo("Adjusting height up")
        key_data.data[8] = 1
    elif cmd == SvanCommand.HEIGHT_DOWN:
        rospy.loginfo("Adjusting height down")
        key_data.data[8] = -1
    key_pub.publish(key_data)

def set_roll(roll: float):
    if current_operation_mode != SvanCommand.TROT:
        rospy.logwarn("Roll commands are only available in trot mode")
        return
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    key_data.data[0] = 4
    constrained_roll = constrain_value(roll, -1.0, 1.0)
    key_data.data[3] = constrained_roll
    key_pub.publish(key_data)

def set_pitch(pitch: float):
    if current_operation_mode != SvanCommand.TROT:
        rospy.logwarn("Pitch commands are only available in trot mode")
        return
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    key_data.data[0] = 4
    constrained_pitch = constrain_value(pitch, -1.0, 1.0)
    key_data.data[4] = constrained_pitch
    key_pub.publish(key_data)

def set_yaw(direction: int, velocity: float):
    if current_operation_mode != SvanCommand.TROT:
        rospy.logwarn("Yaw commands are only available in trot mode")
        return
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    key_data.data[0] = 4
    constrained_velocity = constrain_value(velocity)
    key_data.data[5] = constrained_velocity * (-1 if direction == SvanCommand.YAW_RIGHT else 1)
    key_pub.publish(key_data)

# Subscription
def callback(command: SvanCommand):
    # operation mode
    if command.command_type == SvanCommand.OPERATION_MODE:
        rospy.loginfo(f"Setting operation mode to {command.operation_mode}")
        set_operation_mode(command.operation_mode)

    # linear movement
    elif command.command_type == SvanCommand.COMMAND_MOVEMENT:
        rospy.loginfo(f"Setting movement to {command.direction} with velocity {command.velocity}")
        set_movement(command.direction, command.velocity)
        rospy.loginfo(f"Movement set to {command.direction} with velocity {command.velocity}")
    
    # vertical height
    elif command.command_type == SvanCommand.COMMAND_HEIGHT:
        rospy.loginfo(f"Setting height to {command.height}")
        set_height(command.height)
    
    # roll 
    elif command.command_type == SvanCommand.COMMAND_ROLL:
        rospy.loginfo(f"Setting roll to {command.roll}")
        set_roll(command.roll)

    # pitch
    elif command.command_type == SvanCommand.COMMAND_PITCH:
        rospy.loginfo(f"Setting pitch to {command.pitch}")
        set_pitch(command.pitch)

    # yaw
    elif command.command_type == SvanCommand.COMMAND_YAW:
        rospy.loginfo(f"Setting yaw to {'left' if command.direction == 0 else 'right'} with velocity {command.velocity}")
        set_yaw(command.direction,command.velocity)


sub = rospy.Subscriber('/svan/simple_control', SvanCommand, callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
