#ifndef IO_TURTLE_TURTLE_HPP
#define IO_TURTLE_TURTLE_HPP

#include<string>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <io_turtle_msgs/Pose.h>
#include <io_turtle_msgs/Velocity.h>
#include <io_turtle_msgs/DistanceSensor.h>

namespace turtlesim {

template<class VelMsg, class PoseMsg>
struct TurtleBase {
    TurtleBase() {}

    TurtleBase(int id) {
        set_id(id);
    }

    virtual void set_id(int id) {
        _id = id;
    }

    void set_cmd_vel(VelMsg vel) {
        _cmd_vel = vel;
    }

    virtual void set_cmd_vel_to_current_vel() = 0;
    virtual void set_cmd_vel(float linear, float angular) = 0;    
    virtual float get_linear_velocity() = 0;
    virtual float get_angular_velocity() = 0;
    
    PoseMsg _pose;
    VelMsg _cmd_vel;
    io_turtle_msgs::DistanceSensor _sensors;
    int _id;
};

struct TurtleCustomMsg : TurtleBase<io_turtle_msgs::Velocity, io_turtle_msgs::Pose> {

    TurtleCustomMsg() : TurtleBase(){}
    TurtleCustomMsg(int id) : TurtleBase(id){
        set_id(id);
    }

    void set_id(int id) {
        TurtleBase::set_id(id);
        _pose.id = id;
        _cmd_vel.id = id;
        _sensors.id = id;
    }

    void set_cmd_vel(float linear, float angular) {
        _cmd_vel.linear_velocity = linear;
        _cmd_vel.angular_velocity = angular;
    }

    void set_cmd_vel_to_current_vel() {
        _pose.linear_velocity = _cmd_vel.linear_velocity;
        _pose.angular_velocity = _cmd_vel.angular_velocity;
    }

    float get_linear_velocity(){
        return _pose.linear_velocity;
    }

    float get_angular_velocity(){
        return _pose.angular_velocity;
    }

};

struct TurtleGeometryMsg : TurtleBase<geometry_msgs::Twist, geometry_msgs::Pose2D> {

    TurtleGeometryMsg() : TurtleBase(){}
    TurtleGeometryMsg(int id) : TurtleBase(id){}

    void set_cmd_vel(float linear, float angular) {
        _cmd_vel.linear.x = linear;
        _cmd_vel.angular.z = angular;
    }

    void set_cmd_vel_to_current_vel() {
        _current_vel = _cmd_vel;
    }

    float get_linear_velocity(){
        return _current_vel.linear.x;
    }

    float get_angular_velocity(){
        return _current_vel.angular.z;
    }

   geometry_msgs::Twist _current_vel;

};



}  // namespace turtlesim

#endif /* IO_TURTLE_TURTLE_HPP */
