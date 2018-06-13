import RPi.GPIO as GPIO
from PCA9685 import PCA9685

"""AlphaBot2 class
AlphaBot2 is a small differential drive robot without encoders.

Modified from demo code at https://www.waveshare.com/wiki/AlphaBot2-Pi#Demo
"""
class AlphaBot2(object):
    """initialize bot with proper GPIO pins
    ain1 = GPIO control pin for the left motor (pin 12)
    ain2 = GPIO control pin for the left motor (pin 13)
    ena  = GPIO pin for the left motor (PWM pin 12)
    bin1 = GPIO control pin for the right motor (pin 20)
    bin2 = GPIO control pin for the right motor (pin 21)
    enb  = GPIO pin for the right motor (PWM pin 26)
    """
    def __init__(self, ain1=12, ain2=13, ena=6, bin1=20, bin2=21, enb=26):

        self.wheel_radius = 2.1  # cm
        self.bot_radius = 5.5  # cm

        # these values scale the speed of each motor
        # they should be approximately:
        # 100/(2 * pi * N wheel rotations per second @ max duty cycle)
        left_tune = 1.75
        right_tune = 1.75

        # this value boosts the rotational component of the differential drive
        # in order to compensate for different amounts of slippage while
        # moving forward vs rotating
        rotation_gain = 2

        # the minimum duty cycle % below which there is too little torque to
        # spin at all
        self.min_duty_cycle = 7

        # *NOTE* these values correspond to only when PWM frequency is set to 70Hz.
        # The PWM was intentionally lowered for better torque at lower speeds
        # (low speed corresponds to low torque, which is unable to overcome
        # static friction to start moving unless the current pulses are larger)
        self.tune(left_tune, right_tune, rotation_gain)

        # camera
        self.camera = PCA9685(0x40)
        self.camera.setPWMFreq(50)

        self.camera_hpulse = 1500
        self.camera_hstep = 0
        self.camera.setServoPulse(0, self.camera_hpulse)

        self.camera_vpulse = 1500
        self.camera_vstep = 0
        self.camera.setServoPulse(1, self.camera_vpulse)

        self.AIN1 = ain1
        self.AIN2 = ain2
        self.BIN1 = bin1
        self.BIN2 = bin2
        self.ENA = ena
        self.ENB = enb
        self.PA = 50
        self.PB = 50
        self.BUZ = 4

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.BUZ, GPIO.OUT)

        GPIO.output(self.BUZ, GPIO.LOW)

        self.PWMA = GPIO.PWM(self.ENA, 70)
        self.PWMB = GPIO.PWM(self.ENB, 70)

        self.PWMA.start(self.PA)
        self.PWMB.start(self.PB)

        self.stop()

    def tune(self, left_tune, right_tune, rotation_gain):
        self.velocity_left_constant = (100.0 / self.wheel_radius) * left_tune
        self.velocity_right_constant = (100.0 / self.wheel_radius) * right_tune
        self.angular_left_constant = (self.bot_radius / self.wheel_radius) * left_tune * rotation_gain
        self.angular_right_constant = (self.bot_radius / self.wheel_radius) * right_tune * rotation_gain

    def move(self, linear_x, angular_z):
        linear_component_left = linear_x * self.velocity_left_constant
        angular_component_left = angular_z * self.angular_left_constant
        velocity_left = linear_component_left - angular_component_left

        linear_component_right = linear_x * self.velocity_right_constant
        angular_component_right = angular_z * self.angular_right_constant
        velocity_right = linear_component_right + angular_component_right

        GPIO.output(self.AIN1, GPIO.HIGH if velocity_left < 0 else GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH if velocity_left > 0 else GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.HIGH if velocity_right < 0 else GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH if velocity_right > 0 else GPIO.LOW)

        duty_cycle_left = int(min(100, abs(velocity_left)))
        duty_cycle_right = int(min(100, abs(velocity_right)))

        if duty_cycle_left > 0 and duty_cycle_left < self.min_duty_cycle:
            duty_cycle_left = self.min_duty_cycle

        if duty_cycle_right > 0 and duty_cycle_right < self.min_duty_cycle:
            duty_cycle_right = self.min_duty_cycle

        self.PWMA.ChangeDutyCycle(duty_cycle_left)
        self.PWMB.ChangeDutyCycle(duty_cycle_right)

    def camera_move(self, pitch, yaw):
        self.camera_vpulse += pitch
        if (self.camera_vpulse <= 500):
            self.camera_vpulse = 500
        self.camera.setServoPulse(1, self.camera_vpulse)

        self.camera_hpulse += yaw
        if (self.camera_hpulse >= 2500):
            self.camera_hpulse = 2500
        self.camera.setServoPulse(0, self.camera_hpulse)

    def stop(self):
        self.PWMA.ChangeDutyCycle(0)
        self.PWMB.ChangeDutyCycle(0)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.LOW)
