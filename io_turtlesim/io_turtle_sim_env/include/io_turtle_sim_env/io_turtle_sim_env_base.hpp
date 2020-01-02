#pragma once

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <limits>
#include <unordered_map>

#include <ros/ros.h>

#include <io_turtle/turtle.hpp>
#include <io_turtle_services/RegisterSimTurtle.h>
#include <io_turtle_services/TeleportTurtle.h>

namespace turtlesim {
using io_turtle_services::RegisterSimTurtle;
using io_turtle_services::TeleportTurtle;

template<class Turtle>
class SimTurtleBase {
public:
    SimTurtleBase(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, int id, Turtle turtle, 
            float radius, float collision_range, float x_min, float x_max, float y_min, float y_max)
        :  _id(id), _turtle(turtle), _turtle_radius(radius), _collision_range(collision_range),
        _x_min(x_min), _x_max(x_max), _y_min(y_min), _y_max(y_max){

    };

    void update_pose(float time_step) {
        _turtle.set_cmd_vel_to_current_vel();
        _turtle._pose.theta = std::fmod(_turtle._pose.theta + _turtle.get_angular_velocity() * time_step, 2 * M_PI);

        float delta_pose_x = std::cos(_turtle._pose.theta) * _turtle.get_linear_velocity() * time_step;
        float delta_pose_y = std::sin(_turtle._pose.theta) * _turtle.get_linear_velocity() * time_step;

        float pre_clamped_x = _turtle._pose.x + delta_pose_x;
        float pre_clamped_y = _turtle._pose.y + delta_pose_y;

        _turtle._pose.x = std::fmax(_x_min, std::fmin(_x_max, pre_clamped_x));
        _turtle._pose.y = std::fmax(_y_min, std::fmin(_y_max, pre_clamped_y));

        _pose_pub->publish(_turtle._pose);

    };

    template<class SimTurtle>
    void update_sensor(std::unordered_map<int, SimTurtle>& turtles) {
        float cos_theta = std::cos(_turtle._pose.theta);
        float sin_theta = std::sin(_turtle._pose.theta);

        // Handle boundary distances
        float boundary_distance[4];
        boundary_distance[0] = (cos_theta == 0.0f) ? 0.0f : (_x_min - _turtle._pose.x) / cos_theta;
        boundary_distance[1] = (cos_theta == 0.0f) ? 0.0f : (_x_max - _turtle._pose.x) / cos_theta;
        boundary_distance[2] = (sin_theta == 0.0f) ? 0.0f : (_y_min - _turtle._pose.y) / sin_theta;
        boundary_distance[3] = (sin_theta == 0.0f) ? 0.0f : (_y_max - _turtle._pose.y) / sin_theta;

        std::sort(boundary_distance, boundary_distance + 4);
        _turtle._sensors.rear = boundary_distance[1];
        _turtle._sensors.front = boundary_distance[2];

        // Handle obstacle distances
        for (const std::pair<const int, SimTurtleBase<Turtle>>& turtle_entry : turtles) {
            if (turtle_entry.first == _id)
                continue;

            // Transform coordinates of the turtles to current turtle axis
            float dx = turtle_entry.second._turtle._pose.x - _turtle._pose.x;
            float dy = turtle_entry.second._turtle._pose.y - _turtle._pose.y;
            float tx = dx * cos_theta + dy * sin_theta;
            float ty = -dx * sin_theta + dy * cos_theta;

            // Only consider turtles within collision range
            if (std::fabs(ty) >= _turtle_radius)
                continue;

            // Find the distance to closest obstacle and its position w.r.t current turtle
            float intersection = std::sqrt(_turtle_radius * _turtle_radius - ty * ty);
            if (tx > 0) {
                float distance = std::fmin(tx + intersection, tx - intersection);
                if (distance < _turtle._sensors.front) {
                    _turtle._sensors.front = distance;
                }
            } else if (tx < 0) {
                float distance = std::fmax(tx + intersection, tx - intersection);
                if (distance > _turtle._sensors.rear) {
                    _turtle._sensors.rear = distance;
                }
            }
        }

        _turtle._sensors.rear = std::fabs(_turtle._sensors.rear);

        _sensors_pub->publish(_turtle._sensors);
    }

    bool within_boundary(float x, float y) const {
        return (x >= _x_min + _collision_range) && (x <= _x_max - _collision_range) &&
               (y >= _y_min + _collision_range) && (y <= _y_max - _collision_range);
    }

    void set_cmd_vel(float linear, float angular){
        _turtle.set_cmd_vel(linear, angular);
    }

    bool set_pose(const float x, const float y, const float theta){
        if (!within_boundary(x, y)) {
            ROS_INFO("Turtle cannot be teleported outside boundary.");
            return false;
        }
        _turtle._pose.x = x;
        _turtle._pose.y = y;
        _turtle._pose.theta = theta;
        return true;
    }

protected:
        Turtle _turtle;

        int _id;

        // ROS subscribers and publishers
        ros::Publisher* _pose_pub;
        ros::Publisher* _sensors_pub;

        float _turtle_radius;
        float _collision_range;
        int _x_min;
        int _x_max;
        int _y_min;
        int _y_max;

};

template<class SimTurtleBaseClass>
class SimEnvBase {
public:
    using TurtleMap = std::unordered_map<int, SimTurtleBaseClass>;
    using TurtleEntry = std::pair<const int, SimTurtleBaseClass>;

    SimEnvBase(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, float time_step)
            : _nh(nh), _priv_nh(priv_nh), _time_step(time_step) {
        srand(time(NULL));

        _nh.param("turtle_radius", _turtle_radius, 1.0f);
        _nh.param("collision_range", _collision_range, 2.0f);
        _nh.param("boundary/x_min", _x_min, -10);
        _nh.param("boundary/x_max", _x_max, 10);
        _nh.param("boundary/y_min", _y_min, -10);
        _nh.param("boundary/y_max", _y_max, 10);

        // Construct services
        _register_service = _nh.advertiseService("register_sim_turtle", &SimEnvBase::register_turtle, this);
        ROS_INFO("'Register simulation turtle' service started.");

        _teleport_service = nh.advertiseService("teleport_turtle", &SimEnvBase::teleport_turtle, this);
        ROS_INFO("'Teleport turtle' service started.");
    }

    void update() {
        for (TurtleEntry& turtle : _turtles) {
            turtle.second.update_pose(_time_step);
            turtle.second.update_sensor(_turtles);
            // turtle.second.update_sensor<SimTurtleBaseClass>(_turtles);
        }
    }

    static float random_value(float min, float max) {
        return min + (max - min) * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    }

protected:
    virtual void register_turtle_impl(int id) = 0;
    bool register_turtle(RegisterSimTurtle::Request& req, RegisterSimTurtle::Response& response) {
        response.data = (_turtles.find(req.id) == _turtles.end());
        if(!response.data){
            ROS_ERROR_STREAM("Turtle" << req.id << " already exists in the simulation environment.");
        }
        else{
            register_turtle_impl(req.id);
        }
        return response.data;
    }

    bool teleport_turtle(TeleportTurtle::Request& req, TeleportTurtle::Response& response) {
        typename TurtleMap::iterator turtle_it = _turtles.find(req.id);

        if (turtle_it == _turtles.end()) {
            ROS_INFO("Turtle%d is not registered in simulation.", req.id);
            response.data = false;
            return true;
        }
        
        response.data = turtle_it->second.set_pose(req.x, req.y, req.theta);
        if(response.data){
            ROS_INFO("Teleported Turtle%d to (%f, %f, %f)", req.id, req.x, req.y, req.theta);
        }
        
        return response.data;
    }

    ros::NodeHandle _nh, _priv_nh;
    ros::ServiceServer _register_service;
    ros::ServiceServer _teleport_service;
    float _time_step;
    TurtleMap _turtles;

    float _turtle_radius;
    float _collision_range;
    int _x_min;
    int _x_max;
    int _y_min;
    int _y_max;

};

}  // namespace turtlesim