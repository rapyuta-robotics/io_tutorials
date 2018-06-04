#!/usr/bin/env python

import rospy

from AlphaBot2 import AlphaBot2
from geometry_msgs.msg import Twist
from simon_says_msgs.msg import Mode
from simon_says_msgs.msg import Status

Ab = AlphaBot2()


class RPiGPIO:
    def __init__(self):

        # status (uint8): 0=OFF, 1=ERROR, 2=RUNNING
        # mode (uint8): 0=OFF, 1=SOURCE, 2=SINK, 3=SOURCESINK

        self.status = Status()
        self.status.data = Status.RUNNING
        self.status.mode = Mode()
        self.status.mode.data = Mode.OFF

        self.heartbeat_time = 0.1  # second
        self.previous_instruction_time = 0

        self.init_ros('rpi_gpio')

    def init_ros(self, node_name):
        rospy.init_node(node_name)

        rospy.Subscriber('mode', Mode, self.mode_callback)
        rospy.Subscriber('command', Twist, self.command_callback)
        rospy.Subscriber('camera_move', Twist, self.camera_callback)

        self.status_pub = rospy.Publisher('status', Status, queue_size=1)

        rospy.Timer(rospy.Duration(self.heartbeat_time), self.heartbeat_callback)

        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        Ab.stop()
        self.status.data = Status.OFF
        self.status_pub.publish(self.status)

    def mode_callback(self, msg):
        self.status.mode.data = msg.data
        if self.status.mode.data == Mode.OFF:
            Ab.stop()

    def heartbeat_callback(self, event):
        duration = event.current_real.to_sec() - self.previous_instruction_time
        elapsed_time = rospy.Duration(duration).to_sec()

        if (elapsed_time > self.heartbeat_time):
            Ab.stop()

        self.status_pub.publish(self.status)

    def command_callback(self, msg):
        if self.status.mode.data != Mode.SINK:
            return

        self.previous_instruction_time = rospy.Time.now().to_sec()

        Ab.move(msg.linear.x, msg.angular.z)

    def camera_callback(self, msg):
        if self.status.mode.data != Mode.SOURCESINK or self.status.mode.data != Mode.SINK:
            return

        self.previous_instruction_time = rospy.Time.now().to_sec()

        pitch = int(msg.angular.x)
        yaw = int(msg.angular.z)

        Ab.camera_move(pitch, yaw)

if __name__ == '__main__':
    Rpigpio = RPiGPIO()
