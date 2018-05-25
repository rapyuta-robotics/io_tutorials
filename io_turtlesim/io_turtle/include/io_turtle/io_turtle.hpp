#ifndef IO_TURTLE_IO_TURTLE_HPP
#define IO_TURTLE_IO_TURTLE_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <io_turtle/turtle.hpp>
#include <io_turtle/ControlConfig.h>

namespace turtlesim {

class IOTurtle {
public:
    IOTurtle(ros::NodeHandle& priv_nh);

    void dynamic_parameters_callback(io_turtle::ControlConfig& config, uint32_t level);
    void register_turtle(ros::NodeHandle& nh, int id, bool sim, int queue_size);
    void set_goal(float x, float y);
    void move_towards_goal();
    void stop_turtle();
    void reset();

    bool is_moving() const;
    bool stop_requested() const;
    bool reached_goal() const;
    bool check_collision() const;

private:
    void publish_velocity();
    void velocity_callback(const io_turtle_msgs::Velocity& vel);
    void sim_pose_callback(const io_turtle_msgs::Pose& pose);
    void sim_sensors_callback(const io_turtle_msgs::DistanceSensor& sensor);

    ros::Subscriber _velocity_sub;
    ros::Subscriber _sim_pose_sub;
    ros::Subscriber _sim_sensors_sub;
    ros::Publisher _sim_velocity_pub;
    ros::Publisher _pose_pub;

    Turtle _turtle;
    bool _sim;
    bool _reached_goal;
    bool _stop_requested;
    float _goal_x;
    float _goal_y;
    float _d_0;
    float _collision_range;
    float _goal_tolerance;

    io_turtle::ControlConfig _control_config;
};

}  // namespace turtlesim

#endif /* IO_TURTLE_IO_TURTLE_HPP */
