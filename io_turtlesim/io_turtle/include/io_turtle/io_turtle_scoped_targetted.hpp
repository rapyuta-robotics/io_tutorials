#ifndef IO_TURTLE_IO_TURTLE_SCOPED_TARGETTED_HPP
#define IO_TURTLE_IO_TURTLE_SCOPED_TARGETTED_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <io_turtle/turtle.hpp>
#include <io_turtle/io_turtle_base.hpp>
#include <io_turtle/ControlConfig.h>

namespace turtlesim {

class IOTurtleScopedTargetted : public IOTurtleBase<geometry_msgs::Twist, geometry_msgs::Pose2D> {
public:
    IOTurtleScopedTargetted(ros::NodeHandle& priv_nh);
    ~IOTurtleScopedTargetted();

};

class IOTurtleScopedTargettedNode : public IOTurtleBaseNode<geometry_msgs::Twist, geometry_msgs::Pose2D> {
public:
    IOTurtleScopedTargettedNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    ~IOTurtleScopedTargettedNode();
    void register_turtle();
    void set_dr();
private:
    bool _use_rapyuta_io;
    std::string _name;

};

}  // namespace turtlesim

#endif /* IO_TURTLE_IO_TURTLE_HPP */
