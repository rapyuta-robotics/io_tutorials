#GoPiGo 3 Package

### Description
This package launches the gopigo_control node.

Using the gopigo_control.launch file, the GoPiGo expects to receive geometry_msgs/Twist messages to the /local/cmd_vel topic.

### Testing
Running on a RP3 connected to the GoPiGo shield, with the appropriate packages installed, run the following command:

$ roslaunch gopigo_contol gopigo_control.launch

This should make the GoPiGo drive forward until an obstacle is about 0.5 m away from its distance sensor. If the obstacle approaches, the GoPiGo should back away in order to maintain a 0.5 m separation distance.

In order to test with keyboard teleoperation, run the following command.

$ roslaunch gopigo_contol gopigo_control.launch teleop:=true

In order to expose the twist messages of the GoPiGo device which are reaction information to the obstacles, run the following command.

$ roslaunch gopigo_contol gopigo_control.launch controller:=true
