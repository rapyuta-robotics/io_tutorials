# GPIO package for AlphaBot2-Pi

This contains a ROS package with two nodes: a controller node and a GPIO-manipulation node. The controller node is not necessary, but can be launched on a device with a connected gamepad to control the bot's movement and camera.

Clone this repository into the /src/ directory of your catkin workspace on your device. To deploy the controller: **roslaunch rpi_gpio controller.launch**. To deploy the GPIO node: **roslaunch rpi_gpio rpi_gpio.launch**.

The rpi_gpio_node subscribes to two topics: **/move** and **/camera_move**, both of type **geometry_msgs/Twist**. The Robot's motion is controlled by publishing messages to **/move** and the camera pitch and yaw are controlled by publishing to **/camera_move**.

### **/move**
message.linear.x = m/s forward and backwards

message.angular.z = rad/s left and right turn

### **/camera_move**
message.angular.x = steps pitch (-63 to 63 recommended)

message.angular.z = steps yaw (-63 to 63 recommended)

## Buzzer off on startup
Move the buzzer_off.conf file to /etc/conf/ and move the turn_buzzer_off.sh script to your home directory (/home/ubuntu/).
