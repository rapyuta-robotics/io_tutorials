#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String


def talker():
    rospy.init_node('node', anonymous=True)
    topic_name = rospy.get_param('~topic_name')
    topic_rate = rospy.get_param('~topic_rate')

    pub = rospy.Publisher(topic_name, String, queue_size=10)
    rate = rospy.Rate(topic_rate)

    while not rospy.is_shutdown():
        msg = "talker is publishing at rate: {}".format(topic_rate)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
