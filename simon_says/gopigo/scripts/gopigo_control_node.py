#!/usr/bin/env python

import rospy
import math
import easygopigo3 as easy
import RPi.GPIO as GPIO

from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class GoPiGo(object):
    def __init__(self):
        # Power Management Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(23, GPIO.OUT)
        GPIO.output(23, True)

        # Construct and initialize GoPiGo
        self._gpg = easy.EasyGoPiGo3()
        self._distance_sensor = self._gpg.init_distance_sensor()
        self._servo = self._gpg.init_servo()
        self._prev_deg_L = self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT)
        self._prev_deg_R = self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT)
        self._servo.reset_servo()

        # Bot properties
        self.WHEEL_RADIUS_METRES = self._gpg.WHEEL_DIAMETER / 1000.0 / 2.0
        self.WHEEL_BASE_METRES = self._gpg.WHEEL_BASE_WIDTH / 1000.0
        self.HALF_WHEEL_BASE_METRES = self.WHEEL_BASE_METRES / 2.0
        self.WHEEL_RADIUS_BASE_RATIO = self.WHEEL_RADIUS_METRES / self.WHEEL_BASE_METRES

        # Converts the tangential velocity of the robot's wheel into degrees per second
        self.WHEEL_SPEED_TO_DEG_PER_SEC = math.degrees(1.0 / self.WHEEL_RADIUS_METRES)

        # Converts a change in right wrt left wheel rotation into a change in robot rotation
        self.DEG_DELTA_TO_ROTATION = math.radians(self.WHEEL_RADIUS_BASE_RATIO)

        # Converts a change in left plus right wheel rotations into a change in robot displacement
        self.DEG_SUM_TO_DISPLACEMENT = math.radians(self.WHEEL_RADIUS_METRES / 2)

        # ROS publishers
        self._distance_pub = rospy.Publisher('distance', UInt16, queue_size=1)
        self._odometry_pub = rospy.Publisher('odometry', Twist, queue_size=1)
        self._reaction_pub = rospy.Publisher('reaction', Twist, queue_size=1)
        self._battery_voltage_pub = rospy.Publisher('voltage', Float32, queue_size=1)

        # ROS subscriber
        self._sub = rospy.Subscriber('cmd_vel', Twist, self._command_cb)

        # ROS timers
        distance_freq = rospy.get_param('~distance_update_freq', 20)
        odometry_freq = rospy.get_param('~odometry_update_freq', 30)
        rospy.Timer(rospy.Duration(1.0 / distance_freq), self._distance_update_cb)
        rospy.Timer(rospy.Duration(1.0 / odometry_freq), self._odometry_update_cb)
        rospy.Timer(rospy.Duration(10), self._battery_check_cb)

        self._stale_command_period = rospy.Duration(0.5)
        rospy.Timer(self._stale_command_period, self._stale_command_check)

        # ROS messages
        self._voltage = Float32()
        self._dist_measurement = UInt16()

        # Internal
        self._last_command_timestamp = rospy.Time.now()  # [s]
        self._stopping_distance = 500  # [mm]
        self._margin_distance = 50  # [mm]
        self._approach_speed = 0.1  # [m/s]
        self._retreat_speed = 3 * self._approach_speed  # [m/s]

    def _stale_command_check(self, event):
        if rospy.Time.now() - self._last_command_timestamp > self._stale_command_period:
            self.turn_off_motors()

    def _battery_check_cb(self, event):
        self._voltage.data = self._gpg.volt()
        self._battery_voltage_pub.publish(self._voltage)

    def _distance_update_cb(self, event):
        distance = self._distance_sensor.read_mm()

        self._dist_measurement.data = distance
        self._distance_pub.publish(self._dist_measurement)

        reaction = Twist()

        if distance > (self._stopping_distance + self._margin_distance):
            reaction.linear.x = self._approach_speed
        elif distance < (self._stopping_distance - self._margin_distance):
            reaction.linear.x = -self._retreat_speed

        self._reaction_pub.publish(reaction)

    def _odometry_update_cb(self, event):
        curr_deg_L = self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT)
        curr_deg_R = self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT)

        diff_deg_L = curr_deg_L - self._prev_deg_L
        diff_deg_R = curr_deg_R - self._prev_deg_R

        self._prev_deg_L = curr_deg_L
        self._prev_deg_R = curr_deg_R

        displacement = (diff_deg_L + diff_deg_R) * self.DEG_SUM_TO_DISPLACEMENT

        pose = Twist()

        pose.angular.z += (diff_deg_R - diff_deg_L) * self.DEG_DELTA_TO_ROTATION
        pose.linear.x += displacement * math.cos(pose.angular.z)
        pose.linear.y += displacement * math.sin(pose.angular.z)

        self._odometry_pub.publish(pose)

    def _command_cb(self, msg):
        dps_L = (msg.linear.x - msg.angular.z * self.HALF_WHEEL_BASE_METRES) * self.WHEEL_SPEED_TO_DEG_PER_SEC
        dps_R = (msg.linear.x + msg.angular.z * self.HALF_WHEEL_BASE_METRES) * self.WHEEL_SPEED_TO_DEG_PER_SEC

        self._gpg.set_motor_dps(self._gpg.MOTOR_LEFT, dps_L)
        self._gpg.set_motor_dps(self._gpg.MOTOR_RIGHT, dps_R)

        self._last_command_timestamp = rospy.Time.now()

    def turn_off_motors(self):
        self._gpg.stop()


def main():
    rospy.init_node('gopigo_node')
    gpg = GoPiGo()
    rospy.on_shutdown(gpg.turn_off_motors)
    rospy.spin()


if __name__ == '__main__':
    exit(main())
