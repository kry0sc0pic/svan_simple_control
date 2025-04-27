#!/usr/bin/env python3
from svan_simple_control_msgs.msg import SvanCommand
import rospy

rospy.init_node('rotate_example_node')
c_pub = rospy.Publisher('/svan/simple_control',SvanCommand,queue_size=1)

SIM = True

stop_cmd = SvanCommand()
stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
stop_cmd.operation_mode = SvanCommand.MODE_STOP
c_pub.publish(stop_cmd)

rospy.sleep(2)

print("TROT")
trot_cmd = SvanCommand()
trot_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
trot_cmd.operation_mode = SvanCommand.MODE_TROT
c_pub.publish(trot_cmd)

rospy.sleep(0.1)
print("YAW")
turn_cmd = SvanCommand()
turn_cmd.command_type = SvanCommand.COMMAND_YAW
turn_cmd.yaw = SvanCommand.YAW_RIGHT
c_pub.publish(turn_cmd)

print("SPEED")
rospy.sleep(0.1)
vel_cmd = SvanCommand()
vel_cmd.command_type = SvanCommand.COMMAND_MOVEMENT
vel_cmd.vel_y = 0.0001 if SIM else -0.192
vel_cmd.vel_x = 0.0
c_pub.publish(vel_cmd)

rospy.sleep(20)
print("NO YAW")
no_yaw_cmd = SvanCommand()
no_yaw_cmd.command_type = SvanCommand.COMMAND_YAW
no_yaw_cmd.yaw = SvanCommand.YAW_NONE

c_pub.publish(no_yaw_cmd)

rospy.sleep(0.1)
print("NO SPEED")
no_speed_cmd = SvanCommand()
no_speed_cmd.command_type = SvanCommand.COMMAND_MOVEMENT
no_speed_cmd.vel_x = 0.0
no_speed_cmd.vel_y = 0.0
c_pub.publish(no_speed_cmd)


rospy.sleep(5)
print("STOP")
c_pub.publish(stop_cmd)