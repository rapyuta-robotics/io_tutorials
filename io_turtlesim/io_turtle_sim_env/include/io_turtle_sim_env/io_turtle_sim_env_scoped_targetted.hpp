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

class SimTurtleScopedTargetted : public SimTurtleBase<TurtleGeometryMsg> {
public:
    SimTurtleScopedTargetted(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, int id, TurtleGeometryMsg turtle, 
            float radius, float collision_range, float x_min, float x_max, float y_min, float y_max);
    ~SimTurtleScopedTargetted();
    void velocity_callback(const geometry_msgs::Twist& vel);

private:
    // ROS subscribers and publishers
    ros::Subscriber _velocity_sub;
};


class SimEnvScopedTargetted : public SimEnvBase<SimTurtleScopedTargetted> {
public:
    SimEnvScopedTargetted(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, float time_step);

private:
    void register_turtle_impl(int id);

};

}  // namespace turtlesim