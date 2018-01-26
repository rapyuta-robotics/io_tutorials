#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid


def callback(data):
    rospy.loginfo("Received map data")
    rospy.loginfo("Read a {} X {} map @ {} m/cell"
                  .format(data.info.width,
                          data.info.height,
                          data.info.resolution))


def listener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
