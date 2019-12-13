#ifndef IO_TURTLE_TURTLE_HPP
#define IO_TURTLE_TURTLE_HPP

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <io_turtle_scoped_targetted_msgs/DistanceSensor.h>

namespace turtlesim {

struct Turtle {
    Turtle() {}

    void set_cmd_vel(float linear, float angular) {
        _cmd_vel.linear.x = linear;
        _cmd_vel.angular.z = angular;
    }

    geometry_msgs::Pose2D _pose;
    geometry_msgs::Twist _current_vel;
    geometry_msgs::Twist _cmd_vel;
    io_turtle_scoped_targetted_msgs::DistanceSensor _sensors;
};

}  // namespace turtlesim

#endif /* IO_TURTLE_TURTLE_HPP */
