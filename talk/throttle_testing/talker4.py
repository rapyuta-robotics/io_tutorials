#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String

RATE = float(os.getenv("TOPIC_RATE_4", 10))

def talker4():
	pub = rospy.Publisher('/topic4', String, queue_size=10)
	rospy.init_node('talker4', anonymous=True)
	rate = rospy.Rate(RATE)
	while not rospy.is_shutdown():
		msg = "talker 4 is publishing"
		rospy.loginfo('talker 4 is publishing')
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker4()
	except rospy.ROSInterruptException:
		pass
