# Simon Says central node package
This is intended to be launched as a cloud component on Rapyuta Robotics' io platform, though it can be run locally on a device as well.

### Build, Source, and Launch
Once this repository has been cloned into the src/ directory of your catkin workspace, assuming your working directory is the root of your catkin workspace:
```
catkin build central_node
source devel/setup.bash
roslaunch central_node central_node.launch
```
