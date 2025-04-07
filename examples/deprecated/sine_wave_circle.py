#!/usr/bin/env python3
from svan_simple_control_msgs.msg import SvanCommand
import rospy

rospy.init_node('circle_example')

command_publisher = rospy.Publisher('/svan/simple_control',SvanCommand,queue_size=3)

while not rospy.is_shutdown():
    print("Stopping")
    stop_cmd = SvanCommand()
    stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
    stop_cmd.operation_mode = SvanCommand.MODE_STOP
    command_publisher.publish(stop_cmd)
    rospy.sleep(2)

    print("Switching to Trot Mode")
    command = SvanCommand()
    command.command_type = SvanCommand.COMMAND_OPERATION_MODE
    command.operation_mode = SvanCommand.MODE_TROT
    command_publisher.publish(command)
    rospy.sleep(2)
    
    # cirle commands
    print("Moving in a circle")
    direction_cmd = SvanCommand()
    direction_cmd.command_type = SvanCommand.COMMAND_MOVEMENT
    direction_cmd.direction = SvanCommand.DIRECTION_FORWARD
    direction_cmd.velocity = 0.5

    yaw_cmd = SvanCommand()
    yaw_cmd.command_type = SvanCommand.COMMAND_YAW
    yaw_cmd.direction = SvanCommand.YAW_LEFT
    yaw_cmd.velocity = 1

    command_publisher.publish(direction_cmd)
    rospy.sleep(0.1)
    command_publisher.publish(yaw_cmd)
    rospy.sleep(0.1)
    for i in range(25):
        height_cmd = SvanCommand()
        height_cmd.command_type = SvanCommand.COMMAND_HEIGHT
        if i % 2 == 0:
            height_cmd.height = SvanCommand.HEIGHT_DOWN
        else:
            height_cmd.height = SvanCommand.HEIGHT_UP
        command_publisher.publish(height_cmd)
        rospy.sleep(5)
    
    stop_cmd = SvanCommand()
    stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
    stop_cmd.operation_mode = SvanCommand.MODE_STOP
    command_publisher.publish(stop_cmd)
    rospy.sleep(3)
    sleep_cmd = SvanCommand()
    sleep_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
    sleep_cmd.operation_mode = SvanCommand.MODE_SLEEP
    command_publisher.publish(sleep_cmd)
    rospy.signal_shutdown("completed")


            
