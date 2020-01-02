#pragma once

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <limits>
#include <unordered_map>

#include <ros/ros.h>

#include <io_turtle/turtle.hpp>
#include <io_turtle_sim_env/io_turtle_sim_env_base.hpp>
#include <io_turtle_services/RegisterSimTurtle.h>
#include <io_turtle_services/TeleportTurtle.h>
#include <io_turtle_msgs/DistanceSensor.h>

namespace turtlesim {

class SimTurtle : public SimTurtleBase<TurtleCustomMsg> {
public:
    SimTurtle(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, int id, TurtleCustomMsg turtle, 
            float radius, float collision_range, float x_min, float x_max, float y_min, float y_max,
            ros::Publisher* pose_pub,  ros::Publisher* sensors_pub);
    ~SimTurtle();

private:

};


class SimEnv : public SimEnvBase<SimTurtle> {
public:
    SimEnv(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, float time_step);

private:
    void register_turtle_impl(int id);
    void velocity_callback(const io_turtle_msgs::Velocity& vel);
    ros::Publisher _pose_pub;
    ros::Publisher _sensors_pub;
    ros::Subscriber _velocity_sub;
};

}  // namespace turtlesim