#!/usr/bin/env python3
import os
import sys
from svan_simple_control_msgs.msg import SvanCommand
from std_msgs.msg import Float32MultiArray
import rospy

rospy.init_node('svan_simple_control_node')


key_pub = rospy.Publisher('/svan/io_interface', Float32MultiArray, queue_size=1)

current_operation_mode = -1

def set_operation_mode(mode: int):
    global current_operation_mode
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    if mode == SvanCommand.TROT:
        print("Setting operation mode to trot")
        key_data.data[0] = 4
        current_operation_mode = SvanCommand.TROT
        key_pub.publish(key_data)
    elif mode == SvanCommand.PUSHUP:
        print("Setting operation mode to pushup")
        current_operation_mode = SvanCommand.PUSHUP
        key_data.data[0] = 3
        key_pub.publish(key_data)
    elif mode == SvanCommand.TWIRL:
        print("Setting operation mode to twirl")
        current_operation_mode = SvanCommand.TWIRL
        key_data.data[0] = 2
        key_pub.publish(key_data)
    elif mode == SvanCommand.STOP:
        print("Setting operation mode to stop")
        current_operation_mode = SvanCommand.STOP
        key_data.data[0] = 1
        key_pub.publish(key_data)
    elif mode == SvanCommand.SLEEP:
        print("Setting operation mode to sleep")
        current_operation_mode = SvanCommand.SLEEP
        key_data.data[0] = 6
        key_pub.publish(key_data)

def constrain_movement_velocity(velocity: float):
    return max(0.0, min(velocity, 1.0))

def set_movement(direction: int, velocity: float):
    global current_operation_mode
    if current_operation_mode != SvanCommand.TROT:
        return
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    key_data.data[0] = 4

    if direction == SvanCommand.DIRECTION_FORWARD:
        key_data.data[2] = constrain_movement_velocity(velocity)
    elif direction == SvanCommand.DIRECTION_BACKWARD:
        key_data.data[2] = -1 * constrain_movement_velocity(velocity)
    elif direction == SvanCommand.DIRECTION_LEFT:
        key_data.data[1] = -1 * constrain_movement_velocity(velocity)
    elif direction == SvanCommand.DIRECTION_RIGHT:
        key_data.data[1] = constrain_movement_velocity(velocity)
    elif direction == SvanCommand.DIRECTION_NONE:
        key_data.data[1:] = [0] * 8
    key_pub.publish(key_data)

def set_height(cmd: int):
    if current_operation_mode != SvanCommand.TROT:
        return
    key_data = Float32MultiArray()
    key_data.data = [0] * 9
    key_data.data[0] = 4
    if cmd == SvanCommand.HEIGHT_UP:
        key_data.data[8] = 1
    elif cmd == SvanCommand.HEIGHT_DOWN:
        key_data.data[8] = -1
    key_pub.publish(key_data)


# Subscription
def callback(command: SvanCommand):
    if command.command_type == SvanCommand.OPERATION_MODE:
        print(f"Setting operation mode to {command.operation_mode}")
        set_operation_mode(command.operation_mode)
    elif command.command_type == SvanCommand.COMMAND_MOVEMENT:
        print(f"Setting movement to {command.direction} with velocity {command.velocity}")
        set_movement(command.direction, command.velocity)
        print(f"Movement set to {command.direction} with velocity {command.velocity}")
    elif command.command_type == SvanCommand.COMMAND_HEIGHT:
        print(f"Setting height to {command.height}")
        set_height(command.height)



    # Implement the callback function here

sub = rospy.Subscriber('/svan/simple_control', SvanCommand, callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
