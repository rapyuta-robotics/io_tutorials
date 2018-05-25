#ifndef IO_TURTLE_TURTLE_HPP
#define IO_TURTLE_TURTLE_HPP

#include <io_turtle_msgs/Pose.h>
#include <io_turtle_msgs/Velocity.h>
#include <io_turtle_msgs/DistanceSensor.h>

namespace turtlesim {

struct Turtle {
    Turtle() {}

    Turtle(int id) {
        set_id(id);
    }

    void set_id(int id) {
        _id = id;
        _pose.id = id;
        _cmd_vel.id = id;
        _sensors.id = id;
    }

    void set_cmd_vel(float linear, float angular) {
        _cmd_vel.linear_velocity = linear;
        _cmd_vel.angular_velocity = angular;
    }

    io_turtle_msgs::Pose _pose;
    io_turtle_msgs::Velocity _cmd_vel;
    io_turtle_msgs::DistanceSensor _sensors;
    int _id;
};

}  // namespace turtlesim

#endif /* IO_TURTLE_TURTLE_HPP */
