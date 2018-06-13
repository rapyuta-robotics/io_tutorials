# Simon Says joystick package
This contains a ROS package with a controller node. The controller node can be launched on a device with a connected gamepad and will generate twist messages for a bot's movement and camera.

### Build, Source, and Launch
Once this repository has been cloned into the src/ directory of your catkin workspace, assuming your working directory is the root of your catkin workspace:
```
catkin build joystick
source devel/setup.bash
roslaunch joystick joystick.launch
```

The joystick node subscribes to one topic: **/joystick/mode** of custom message type Mode, which contains a uint8 enumeration of modes (OFF, SOURCE, SINK, SOURCESINK) that determine if the controller is able to publish messages or not.

The joystick node publishes to two topics: **/joystick/status** and **/joystick/command_req**, which are of custom type Status and type **geometry_msgs/Twist**, respectively. Status contains a uint8 enumeration of statuses (OFF, ERROR, RUNNING) indicating node health.

Only the linear x and angular z components of the **/joystick/command_req** twist message are used as follows:

message.linear.x = m/s forward and backwards

message.angular.z = rad/s left and right turn
