#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os

def talker():
	pub = rospy.Publisher('telemetry', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		hello_str = "hello_world {} {} {} {}".format(os.getenv('RIO_DEVICE_UUID'), os.getenv('RIO_DEVICE_NAME_LAST_SEEN'), os.getenv('RIO_PROJECT_UUID'), os.getenv('RIO_ROS_ENV_ALIAS'))
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
