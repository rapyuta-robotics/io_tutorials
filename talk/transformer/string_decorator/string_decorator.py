#!/usr/bin/env python

import rospy
from std_msgs.msg import String

decoration = " Rapyuta Robotics "
pub = None

def callback(data):
    global decoration
    global pub
    decorated_str = "%s %s" % (decoration, data.data)
    pub.publish(decorated_str)
    rospy.loginfo(decorated_str)

def string_decorator():
    global pub
    rospy.init_node('string_decorator', anonymous=True)
    pub = rospy.Publisher('chatter_decorated', String, queue_size=10)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        string_decorator()
    except rospy.ROSInterruptException:
        pass
