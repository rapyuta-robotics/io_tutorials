#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)


def listener():
	topicname=os.getenv('topic_name','telemetry')
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber(topicname, String, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()
