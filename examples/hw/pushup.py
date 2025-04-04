#!/usr/bin/env python3
from svan_simple_control_msgs.msg import SvanCommand
import rospy

rospy.init_node('pushup_example_node')

c_pub = rospy.Publisher('/svan/simple_control',SvanCommand,queue_size=1)

print("Stopping")

stop_cmd = SvanCommand()
stop_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
stop_cmd.operation_mode = SvanCommand.MODE_STOP
c_pub.publish(stop_cmd)
# c_pub.publish(stop_cmd)

rospy.sleep(2)

print("PUSHUP")
pup_cmd = SvanCommand()
pup_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
pup_cmd.operation_mode = SvanCommand.MODE_PUSHUP
c_pub.publish(pup_cmd)

rospy.sleep(10)
print("TWIRL")
twirl_cmd = SvanCommand()
twirl_cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
twirl_cmd.operation_mode = SvanCommand.MODE_TWIRL

rospy.sleep(10)
print("STOP")
c_pub.publish(stop_cmd)
