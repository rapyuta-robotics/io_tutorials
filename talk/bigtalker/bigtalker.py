#!/usr/bin/env python
import os

import rospy
from std_msgs.msg import String

MB = int(os.getenv("MB", 8))
TIMES = int(os.getenv("TIMES", 1))

def talker():
	HELLO_STR = ''
	for i in range(MB * 100000):
		HELLO_STR = HELLO_STR + '1234567890'

	pub = rospy.Publisher('telemetry', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	i = 0
	while i < TIMES:
		hello_str = "%s MB: %s" % (HELLO_STR, MB)
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		i = i + 1

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
