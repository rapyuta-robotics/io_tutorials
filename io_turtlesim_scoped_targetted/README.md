# IO Turtle Simulation

### Setup (optional)
Set the following environment variables if you want to override the default settings of port 9090 and address 0.0.0.0:
```
export WS_PORT=9090
export WS_ADDR="0.0.0.0"
```

### Building the packages
Clone or symlink this directory into the /src directory of your catkin workspace, then build and source.
Assuming your current directory is the root of your catkin workspace:
```
catkin build io_turtle_sim_env
source devel/setup.bash
```

### Launching simulation locally
After successfully building and sourcing the project, launch it:
```
roslaunch io_turtle_sim_env demo_sim.launch
```

The UI can be found in the [_io\_turtlesim\_webserver_](io_turtlesim_webserver/src/index.html) package. 
