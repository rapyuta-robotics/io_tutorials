#ifndef IO_TURTLE_IO_TURTLE_HPP
#define IO_TURTLE_IO_TURTLE_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <io_turtle/turtle.hpp>
#include <io_turtle/io_turtle_base.hpp>
#include <io_turtle/ControlConfig.h>

namespace turtlesim {

class IOTurtle : public IOTurtleBase<io_turtle_msgs::Velocity, io_turtle_msgs::Pose> {
public:
    IOTurtle(ros::NodeHandle& priv_nh);
    ~IOTurtle();

    void register_turtle(ros::NodeHandle& nh, int id, bool sim, int queue_size);

private:
    void publish_velocity();
    void velocity_callback(const io_turtle_msgs::Velocity& vel);
    void sim_pose_callback(const io_turtle_msgs::Pose& pose);
    void sim_sensors_callback(const io_turtle_msgs::DistanceSensor& sensor);

};

class IOTurtleNode : public IOTurtleBaseNode<io_turtle_msgs::Velocity, io_turtle_msgs::Pose> {
public:
    IOTurtleNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    ~IOTurtleNode();
    void register_turtle();
    void set_dr();
private:
    void set_action_name();
};

}  // namespace turtlesim

#endif /* IO_TURTLE_IO_TURTLE_HPP */
