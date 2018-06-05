#!/usr/bin/env python

import rospy
import thread
import math
import inputs

from geometry_msgs.msg import Twist
from simon_says_msgs.msg import Mode
from simon_says_msgs.msg import Status


class Joystick():
    def __init__(self, rate=20):

        # status: 0=OFF, 1=ERROR, 2=RUNNING
        # mode: 0=OFF, 1=SOURCE, 2=SINK, 3=SOURCESINK

        self.status = Status()
        self.status.data = Status.RUNNING
        self.status.mode = Mode()
        self.status.mode.data = Mode.OFF

        self.heartbeat_time = 1  # second

        self.init_ros('joystick', rate)

        thread.start_new_thread(self.publish_messages, ('message publisher', 1))

    def init_ros(self, node_name, rate):
        rospy.init_node(node_name)

        rospy.Subscriber('mode', Mode, self.mode_callback)

        self.status_pub = rospy.Publisher('status', Status, queue_size=1)
        self.movement_pub = rospy.Publisher('command_req', Twist, queue_size=1)
        self.camera_pub = rospy.Publisher('camera_move', Twist, queue_size=1)

        rospy.Timer(rospy.Duration(self.heartbeat_time), self.heartbeat_callback)

        self.rate = rospy.Rate(rate)  # Hz

        self.movement_msg = Twist()
        self.camera_msg = Twist()

        rospy.on_shutdown(self.shutdown)

    def mode_callback(self, msg):
        self.status.mode.data = msg.data

    def heartbeat_callback(self, event):
        self.status_pub.publish(self.status)

    def shutdown(self):
        self.status.data = Status.OFF
        self.status_pub.publish(self.status)

    def publish_messages(self, *args):
        while not rospy.is_shutdown():
            if self.status.mode.data is Mode.SOURCE or self.status.mode.data is Mode.SOURCESINK:
                self.movement_pub.publish(self.movement_msg)
                self.camera_pub.publish(self.camera_msg)
            self.rate.sleep()

    def control(self):
        while not rospy.is_shutdown():
            try:
                events = inputs.get_gamepad()
            except (IOError, inputs.UnpluggedError) as ioe:
                print("{}".format(ioe))
                events = []
                inputs.devices = inputs.DeviceManager()
                rospy.sleep(0.1)

            for event in events:
                if event.ev_type is 'Absolute':
                    if event.code is 'ABS_Y':
                        linear_velocity = -1.0 * \
                            (((event.state * 2.0) / 255) - 1)
                        if abs(linear_velocity) < 0.05:
                            linear_velocity = 0
                        self.movement_msg.linear.x = linear_velocity
                    elif event.code is 'ABS_X':
                        angular_velocity = ((event.state * 2.0) / 255) - 1
                        if abs(angular_velocity) < 0.05:
                            angular_velocity = 0
                        self.movement_msg.angular.z = - \
                            angular_velocity * math.pi
                    elif event.code is 'ABS_Z':
                        yaw = -1 * ((event.state / 4) - 31)
                        self.camera_msg.angular.z = yaw
                    elif event.code is 'ABS_RZ':
                        pitch = ((event.state / 4) - 31)
                        self.camera_msg.angular.x = pitch


if __name__ == '__main__':
    joystick = Joystick()
    joystick.control()
