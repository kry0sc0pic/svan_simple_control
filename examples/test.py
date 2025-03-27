#!/usr/bin/env python3
import rospy
from svan_simple_control_msgs.msg import SvanCommand

pub = rospy.Publisher('/svan/simple_control', SvanCommand, queue_size=1)

rospy.init_node('svan_simple_control_pub')


def main():
    while not rospy.is_shutdown():
        print('Setting operation mode to trot')
        command = SvanCommand()
        command.command_type = SvanCommand.OPERATION_MODE
        command.operation_mode = SvanCommand.PUSHUP
        pub.publish(command)
        rospy.sleep(10)
        command = SvanCommand()
        command.command_type = SvanCommand.OPERATION_MODE
        command.operation_mode = SvanCommand.SLEEP
        pub.publish(command)
        rospy.sleep(10)
        continue
        command = SvanCommand()
        command.command_type = SvanCommand.COMMAND_MOVEMENT
        command.direction = SvanCommand.DIRECTION_FORWARD
        command.velocity = 0.5
        pub.publish(command)
        rospy.sleep(10)
        command = SvanCommand()
        command.command_type = SvanCommand.COMMAND_MOVEMENT
        command.direction = SvanCommand.DIRECTION_NONE
        pub.publish(command)
        rospy.sleep(10)
       
        


if __name__ == '__main__':
    main()
    