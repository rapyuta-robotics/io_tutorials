#!/usr/bin/env python

import rospy
from rospy import get_param
from std_msgs.msg import String

def talker(first_name, last_name):
	pub = rospy.Publisher('telemetry', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		hello_str = 'hello {} {} {}'.format(first_name, last_name, rospy.get_time())
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	first_name, last_name = get_param('name/first_name'), get_param('name/last_name')
	try:
		talker(first_name, last_name)
	except rospy.ROSInterruptException:
		pass
