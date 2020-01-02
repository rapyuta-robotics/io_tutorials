#include <cmath>
#include <cstdlib>
#include <ctime>

#include <limits>
#include <unordered_map>

#include <ros/ros.h>

#include <io_turtle/turtle.hpp>
#include <io_turtle_sim_env/io_turtle_sim_env.hpp>
#include <io_turtle_sim_env/io_turtle_sim_env_scoped_targetted.hpp>
// #include <io_turtle_sim_env/io_turtle_sim_env_base.hpp>

int main(int argc, char** argv) {
    bool scoped_targetted = false;
    if(argc>1 and std::string(argv[1]) == "--scoped_targetted"){
        scoped_targetted = true;
    }

    // Initialize ROS
    ros::init(argc, argv, "io_turtle_sim_env_node");

    float time_step;

    // Load parameters
    ros::NodeHandle nh, priv_nh("~");
    priv_nh.param("time_step", time_step, 0.1f);

    if(scoped_targetted){
        ROS_INFO("Started Simulation Environment Node with Scoped and Targetted.");
        turtlesim::SimEnvScopedTargetted sim_env(nh, priv_nh, time_step);

        // Set ROS loop rate
        ros::Rate rate(1.0 / time_step);
        while (ros::ok()) {
            sim_env.update();
            ros::spinOnce();
            rate.sleep();
        }
    }
    else{
        ROS_INFO("Started Simulation Environment Node.");
        turtlesim::SimEnv sim_env(nh, priv_nh, time_step);

        // Set ROS loop rate
        ros::Rate rate(1.0 / time_step);
        while (ros::ok()) {
            sim_env.update();
            ros::spinOnce();
            rate.sleep();
        }

    }

    return 0;
}
