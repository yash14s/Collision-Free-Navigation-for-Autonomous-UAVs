#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8

def callback(cmd_msg):
    rospy.loginfo("Command received: %s",cmd_msg.data)

rospy.init_node('cmd_sub', anonymous=True)
rospy.Subscriber("cmd_topic", Int8, callback)
rospy.spin()


