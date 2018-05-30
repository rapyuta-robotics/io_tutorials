#!/usr/bin/env python

import rospy
import math
import easygopigo3 as easy

from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class GoPiGoWrapper(object):
    """ GoPiGoWrapper wraps some of the Python SDK for the GoPiGo3 into a ROS node.

    Params:
        distance_update_freq (int): update frequency (Hz) of the forward-facing distance sensor
        odometry_update_freq (int): update frequency (Hz) of the pose estimate based on odometry

    Publications:
        distance (UInt16): reports the distance to the nearest forward obstacle in mm
        odometry (Twist): reports estimated robot pose calculated using odometry

    Subscriptions:
        cmd_vel (Twist): sets a desired linear and angular velocity
    """

    def __init__(self):
        """ Constructor.
        """
        self._distance_freq = rospy.get_param('~distance_update_freq', 20)
        self._odometry_freq = rospy.get_param('~odometry_update_freq', 30)

        self._stopping_distance = 500  # [mm]

        self._gpg = easy.EasyGoPiGo3()
        self._distance_sensor = self._gpg.init_distance_sensor()
        self._servo = self._gpg.init_servo()

        self.WHEEL_RADIUS_METRES = self._gpg.WHEEL_DIAMETER / 1000.0 / 2.0
        self.WHEEL_BASE_METRES = self._gpg.WHEEL_BASE_WIDTH / 1000.0
        self.HALF_WHEEL_BASE_METRES = self.WHEEL_BASE_METRES / 2.0
        self.WHEEL_RADIUS_BASE_RATIO = self.WHEEL_RADIUS_METRES / \
            self.WHEEL_BASE_METRES

        #: Converts the tangential velocity of the robot's wheel into degrees per second
        self.WHEEL_SPEED_TO_DEG_PER_SEC = math.degrees(1.0 / self.WHEEL_RADIUS_METRES)

        #: Converts a change in right wrt left wheel rotation into a change in robot rotation
        self.DEG_DELTA_TO_ROTATION = math.radians(self.WHEEL_RADIUS_BASE_RATIO)

        #: Converts a change in left plus right wheel rotations into a change in robot displacement
        self.DEG_SUM_TO_DISPLACEMENT = math.radians(self.WHEEL_RADIUS_METRES / 2)

        self._distance_pub = rospy.Publisher('distance', UInt16, queue_size=1)
        self._odometry_pub = rospy.Publisher('odometry', Twist, queue_size=1)
        self._reaction_pub = rospy.Publisher('reaction', Twist, queue_size=1)
        self._battery_voltage_pub = rospy.Publisher('voltage', Float32, queue_size=1)

        self._sub = rospy.Subscriber('cmd_vel', Twist, self._command_cb)

        self._distance_timer = rospy.Timer(
            rospy.Duration(1. / self._distance_freq), self._distance_update_cb
        )
        self._odometry_timer = rospy.Timer(
            rospy.Duration(1. / self._odometry_freq), self._odometry_update_cb
        )

        self._pose = Twist()

        self._prev_deg_L = self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT)
        self._prev_deg_R = self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT)
        self._servo.reset_servo()

        self._last_command_timestamp = rospy.Time.now()
        self._stale_command_period = rospy.Duration(0.5)
        rospy.Timer(self._stale_command_period, self._stale_command_check)

        self._battery_check_period = rospy.Duration(10)
        rospy.Timer(self._battery_check_period, self._battery_check_cb)

    def _stale_command_check(self, event):
        """Turns off motors if last command was received more than self _stale_command_period ago.
        """

        if rospy.Time.now() - self._last_command_timestamp > self._stale_command_period:
            self.turn_off_motors()

    def _battery_check_cb(self, event):
        """Publishes the voltage of the battery.
        """

        voltage_msg = Float32()

        voltage_msg.data = self._gpg.volt()

        self._battery_voltage_pub.publish(voltage_msg)

    def _distance_update_cb(self, event):
        """Read from the distance sensor and publish the result to the /distance topic.

        Distance sensor readings represent the distance in mm to the nearest obstacle in the
        forward direction. When the sensor is unable to detect an obstacle it will report 3000mm

        This function is also responsible for calculating a reaction Twist message, which if
        republished to the device would have the effect of closed-looped position control
        to the self._stopping_distance setpoint.

        Args:
            event (rospy.TimerEvent): an unused event object passed by rospy.Timer

        Returns:
            None

        """

        distance = self._distance_sensor.read_mm()
        dist_measurement_msg = UInt16()
        dist_measurement_msg.data = distance
        self._distance_pub.publish(dist_measurement_msg)

        reaction_msg = Twist()

        approach_speed = 0.1  # [m/s]
        retreat_speed = 3 * approach_speed  # makes behaviour more lifelike

        margin_distance = 50  # [mm]

        if distance > (self._stopping_distance + margin_distance):
            # too far, move closer
            reaction_msg.linear.x = approach_speed
        elif distance < (self._stopping_distance - margin_distance):
            # too close, move away
            reaction_msg.linear.x = -3 * retreat_speed

        self._reaction_pub.publish(reaction_msg)

    def _odometry_update_cb(self, event):
        """Read the wheel encoders and publish the updated odometry pose to the /odometry topic

        Odometry pose is published as a Twist message where linear.x and linear.y represents the x
        and y displacement of the robot in metres and angular.z represents its rotation in radians.

        Args:
            event (rospy.TimerEvent): an unused event object passed by rospy.Timer

        Returns:
            None

        """
        curr_deg_L = self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT)
        curr_deg_R = self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT)

        d_deg_L = curr_deg_L - self._prev_deg_L
        d_deg_R = curr_deg_R - self._prev_deg_R

        displacement = (d_deg_L + d_deg_R) * self.DEG_SUM_TO_DISPLACEMENT
        self._pose.angular.z += (d_deg_R - d_deg_L) * \
            self.DEG_DELTA_TO_ROTATION

        self._pose.linear.x += displacement * math.cos(self._pose.angular.z)
        self._pose.linear.y += displacement * math.sin(self._pose.angular.z)

        self._prev_deg_L = curr_deg_L
        self._prev_deg_R = curr_deg_R

        self._odometry_pub.publish(self._pose)

    def _command_cb(self, msg):
        """Set the robot's motor speeds to achieve a desired linear and angular velocity

        Velocity commands are received from the /cmd_vel subscription as a Twist object.
        The linear.x and angular.z members of Twist are taken as the desired linear and
        angular velocity in m/s and rad/s respectively (all other fields are ignored).

        Note: Wheel speed saturates at 1000 dps.

        Args:
            msg (geometry_msgs.Twist): the velocity command

        Returns:
            None

        """
        dps_L = (
            (msg.linear.x - msg.angular.z * self.HALF_WHEEL_BASE_METRES) *
            self.WHEEL_SPEED_TO_DEG_PER_SEC
        )
        dps_R = (
            (msg.linear.x + msg.angular.z * self.HALF_WHEEL_BASE_METRES) *
            self.WHEEL_SPEED_TO_DEG_PER_SEC
        )

        self._gpg.set_motor_dps(self._gpg.MOTOR_LEFT, dps_L)
        self._gpg.set_motor_dps(self._gpg.MOTOR_RIGHT, dps_R)

        self._last_command_timestamp = rospy.Time.now()

    def turn_off_motors(self):
        """Stops the GoPiGo's motors.
        """
        self._gpg.stop()


def main():
    rospy.init_node('gopigo_control_node')
    gpg = GoPiGoWrapper()
    rospy.on_shutdown(gpg.turn_off_motors)
    rospy.spin()


if __name__ == '__main__':
    exit(main())
