#!/usr/bin/env python3
from svan_simple_control.msg import SvanCommand
from std_msgs.msg import Float32MultiArray
import rospy

rospy.init_node('svan_simple_control_node')

command_publisher = rospy.Publisher('/svan/io_interface',Float32MultiArray,queue_size=1)


def override_listener(msg: Float32MultiArray):
    global failsafe
    if msg.data[7] != 1000 and not failsafe:
        rospy.logerr("Manual Override")
        failsafe = True


base_data = [0,0,0,0,0,0,0,1000,0]
current_operation_mode = SvanCommand.MODE_STOP
current_key_data = Float32MultiArray()
current_key_data.data = base_data
failsafe = False

def constrain_value(value,minumum: float = -1.0,maximum: float = 1.0):
    return max(minumum, min(value, maximum))


def set_operation_mode(mode: int):
    global current_operation_mode, current_key_data, command_publisher
    current_key_data.data = base_data
    if mode == SvanCommand.MODE_TROT:
        rospy.loginfo("MODE: Trot")
        current_operation_mode = SvanCommand.MODE_TROT
        current_key_data.data[0] = 4.0
    elif mode == SvanCommand.MODE_PUSHUP:
        rospy.loginfo("MODE: PushUp")
        current_operation_mode = SvanCommand.MODE_PUSHUP
        current_key_data.data[0] = 3.0
    elif mode == SvanCommand.MODE_TWIRL:
        rospy.loginfo("MODE: Twirl")
        current_operation_mode = SvanCommand.MODE_TWIRL
        current_key_data.data[0] = 2.0
    elif mode == SvanCommand.MODE_STOP:
        rospy.loginfo("MODE: Stop")
        current_operation_mode = SvanCommand.MODE_STOP
        current_key_data.data[0] = 1.0
    elif mode == SvanCommand.MODE_SLEEP:
        rospy.loginfo("MODE: Sleep")
        current_operation_mode = SvanCommand.MODE_SLEEP
        current_key_data.data[0] = 6.0

    command_publisher.publish(current_key_data)

def set_velocity(vel_x: float = 0.0, vel_y: float = 0.0):
    global current_operation_mode, current_key_data
    if vel_x == 0.0 and vel_y == 0.0:
        if current_operation_mode == SvanCommand.MODE_TROT:
            set_operation_mode(SvanCommand.MODE_STOP)
        return

    if current_operation_mode != SvanCommand.MODE_TROT:
        set_operation_mode(SvanCommand.MODE_TROT)
    
    vel_x = constrain_value(vel_x)
    vel_y = constrain_value(vel_y)
    rospy.loginfo(f"Velocity: Vel_X: {vel_x} Vel_Y: {vel_y}")
    current_key_data.data[1] = vel_x
    current_key_data.data[2] = vel_y
    command_publisher.publish(current_key_data)

def set_roll(magnitude: float):
    global current_key_data
    magnitude = constrain_value(magnitude)
    current_key_data.data[3] = magnitude
    rospy.loginfo(f"Roll: {magnitude}")
    command_publisher.publish(current_key_data)

def set_pitch(magnitude: float):
    global current_key_data
    magnitude = constrain_value(magnitude)
    current_key_data.data[4] = magnitude
    rospy.loginfo(f"Pitch: {magnitude}")
    command_publisher.publish(current_key_data)

def set_yaw(direction: int = SvanCommand.YAW_NONE):
    global current_key_data
    if direction == SvanCommand.YAW_NONE and current_key_data.data[5] != 0:
        current_key_data.data[5] = 0
        rospy.loginfo("Yaw: None")
        command_publisher.publish(current_key_data)
    
    elif direction == SvanCommand.YAW_RIGHT and current_key_data.data[5] != -1.0:
        current_key_data.data[5] = -1.0
        rospy.loginfo("Yaw: Right")
        command_publisher.publish(current_key_data)
    
    elif direction == SvanCommand.YAW_RIGHT and current_key_data.data[5] != 1.0:
        current_key_data.data[5] = 1.0
        rospy.loginfo("Yaw: Left")
        command_publisher.publish(current_key_data)

def set_height(state: int):
    global current_key_data
    
    if state == SvanCommand.HEIGHT_UP and current_key_data.data[8] != 1:
        current_key_data.data[8] = 1
        rospy.loginfo("Height: Up")
        command_publisher.publish(current_key_data)
    
    elif state == SvanCommand.HEIGHT_DOWN and current_key_data.data[8] != -1:
        current_key_data.data[8] = -1
        rospy.loginfo("Height: Down")
        command_publisher.publish(current_key_data)

def handle_new_command(command: SvanCommand):
    global failsafe
    rospy.logdebug(command)
    if failsafe:
        rospy.logwarn("Failsafe Triggered. Ignoring command.")
        return
    # operation mode
    if command.command_type == SvanCommand.COMMAND_OPERATION_MODE:
        rospy.logdebug(f"Setting operation mode to {command.operation_mode}")
        set_operation_mode(mode=command.operation_mode)

    # linear movement
    elif command.command_type == SvanCommand.COMMAND_MOVEMENT:
        rospy.logdebug(f"Setting Velocity: Vel_X: {command.vel_x} Vel_Y: {command.vel_y}")
        set_velocity(vel_x=command.vel_x,vel_y=command.vel_y)

    # vertical height
    elif command.command_type == SvanCommand.COMMAND_HEIGHT:
        rospy.logdebug(f"Setting height to {command.height}")
        set_height(state=command.height)
    
    # roll 
    elif command.command_type == SvanCommand.COMMAND_ROLL:
        rospy.logdebug(f"Setting roll to {command.roll}")
        set_roll(magnitude=command.roll)

    # pitch
    elif command.command_type == SvanCommand.COMMAND_PITCH:
        rospy.logdebug(f"Setting pitch to {command.pitch}")
        set_pitch(magnitude=command.pitch)

    # yaw
    elif command.command_type == SvanCommand.COMMAND_YAW:
        rospy.logdebug(f"Setting yaw to {command.yaw}")
        set_yaw(direction=command.yaw)

rospy.Subscriber('/svan/simple_control',SvanCommand,handle_new_command)
rospy.Subscriber('/svan/io_interface',Float32MultiArray,override_listener)

if __name__ == '__main__':
    try:
        rospy.loginfo("READY")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
