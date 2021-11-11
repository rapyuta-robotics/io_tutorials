#!/usr/bin/env python

import random

import rospy
from std_msgs.msg import String

from ros_monitoring_msgs.msg import MetricList, MetricData, MetricDimension


def get_metric_list(cycle, count):
    robot_dimensions = [
        MetricDimension(name='cycle', value='cycle' + str(cycle)),
        MetricDimension(name='random_tag', value=str(random.choice([0, 1]))),
    ]
    return [
        MetricData(
            metric_name='robot.battery_charge',
            unit=MetricData.UNIT_PERCENTAGE,
            value=100 - (count * 10),
            dimensions=robot_dimensions,
        ),
        MetricData(
            metric_name='robot.distance_traveled',
            unit='meters',
            value=random.uniform(count * 100.0, (count+1) * 100.0),
            dimensions=robot_dimensions,
        ),
        MetricData(
            metric_name='edge.connected_robots',
            unit=MetricData.UNIT_COUNT,
            value=random.randint(1, 100),
        ),
    ]


def publish():
    pub = rospy.Publisher('/io_metrics', MetricList, queue_size=10)
    rospy.init_node('metric_publisher', anonymous=True)
    rate = rospy.Rate(0.2)
    cycle = 1
    count = 1
    while not rospy.is_shutdown():
        pub.publish(MetricList(get_metric_list(cycle, count)))
        rospy.loginfo('published metric list for cycle: %d, count: %d', cycle, count)
        rate.sleep()
        if count == 10:
            cycle = 1 if cycle == 10 else cycle + 1
            count = 1  # reset
        else:
            count += 1


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
