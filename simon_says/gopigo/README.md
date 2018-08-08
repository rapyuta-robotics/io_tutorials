# Simon Says gopigo package

### Prerequisites
Raspberry Pi 3 connected to the GoPiGo shield.
#### GoPiGo3 Platform
```
curl -kL dexterindustries.com/update_gopigo3 | sudo bash
curl -kL dexterindustries.com/update_sensors | sudo bash
sudo pip install RPi.GPIO
```

### Build, Source, and Launch
Once this repository has been cloned into the src/ directory of your catkin workspace, assuming your working directory is the root of your catkin workspace:
```
catkin build gopigo
source devel/setup.bash
export GOPIGO_LEADER=true
roslaunch gopigo gopigo.launch
```

This should make the GoPiGo drive forward until an obstacle is about 0.5m away from its distance sensor. If the obstacle approaches, the GoPiGo should back away in order to maintain a 0.5m distance.

The value of the environment variable GOPIGO_LEADER - true or false - determines whether the GoPiGo uses its own distance sensors to control its body or whether it must receive velocity commands externally, respectively. If GOPIGO_LEADER is not set, the GoPiGo will be a follower by default.

#### Notes
Make sure the board is supplied with 9V via the power connector on the GoPiGo shield. If the board is supplied with insufficient power, the motors will not run.
