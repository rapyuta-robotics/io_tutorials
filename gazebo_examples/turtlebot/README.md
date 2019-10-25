# IO Gazebo Turtlebot

### Overview
Sample ROS packages for running turtlebot3 simulation with rapyuta.io.
You can run gazebo in the rapyuta.io and app, e.g. navigation in the your laptop or other cloud instance.

### Packages
- `io_gazebo_turtlebot_bringup` : include launch file which run sim and app separately
- `io_gazebo_turtlebot_description`: include launch file which start gazebo and descriptin which required for app and sim. Urdf, mesh and gazebo plugin sources should be in this packages.
- `io_gazebo_turtlebot_navigation`: include navigation launch file and config file
- `io_gazebo_turtlebot_demo_app`: include sample app which publish sequential move_base goal. 
### Build
#### prep
- [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

#### Build steps
*please replace catkin_ws with your own workspace
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/rapyuta-robotics/io_tutorials.git
sudo rosdep init
rosdep update
rosdep install --from-paths --ignore-src . -y
cd ../
catkin build
```

### Run
#### Run in the local machine
```
source catkin_ws/devel/setup.bash
roslaunch io_gazebo_turtlebot_bringup bringup.launch 
```

#### Run with rapyuta.io
Run gazebo on rapyuta.io and app, e.g. navigation on your device.
This steps require knowledge on rapyuta.io. Reccomend you to go through the [rapyuta.io developer tutorial](https://userdocs.rapyuta.io/dev-tutorials/).
##### rapyuta.io 
By following the [tutorial](https://userdocs.rapyuta.io/dev-tutorials/turtlebot-simulation/) to create package for simulation. When creating package you need to change following parts
1. [Turtlebot3 Robot Simulation Package](https://userdocs.rapyuta.io/dev-tutorials/turtlebot-simulation/#turtlebot3-robot-simulation-package)
    - change git repository to https://github.com/rapyuta-robotics/io_tutorials.git
    - change executable cmd to `roslaunch io_gazebo_turtlebot_bringup sim.launch`
    - add `/odom`, `/scan`, `/tf` and `/joint_states` to ROS topic interface which is need to be shared among navigation and gazebo
    - add `/cmnd_vel` to ROS Inboud ropic interface which is send from navigation to gazebo
    - Instead of 
2. Turtlebot3 app Package
Instead of createing [Turtlebot3 Keyboard Teleoperation Package](https://userdocs.rapyuta.io/dev-tutorials/turtlebot-simulation/#turtlebot3-keyboard-teleoperation-package), you need to create package which start roscore in your device. I'll not explain in detail here. Please check rapyuta.io documantation. Brief steps are
    - Onboard your device which will run `app.launch` to rapyuta.io.  [rapyuta.io docs: Add new device](https://userdocs.rapyuta.io/getting-started/add-new-device)
    - Create package which 
        - device runtime
        - executable cmd: `roscore`, type: `default`
        - rostopic interface: `/cmd_vel`
3. Deploy
    i) deploy Turtlebot3 Robot Simulation package.
    ii) deploy TUrtlebot3 app package with dependent on  deployment of Turtlebot3 Robot simulation package.

##### device
- launch the app in your device.
`roslaunch io_gazebo_turtlebot_bringup app.launch`
you will see the turtlebot start moving in rviz in your device and gazebo view in the cloud

