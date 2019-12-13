#include <cmath>
#include <cstdlib>
#include <ctime>

#include <limits>
#include <unordered_map>

#include <ros/ros.h>

#include <io_turtle_scoped_targetted/turtle.hpp>
#include <io_turtle_scoped_targetted_services/RegisterSimTurtle.h>
#include <io_turtle_scoped_targetted_services/TeleportTurtle.h>

namespace turtlesim {

using io_turtle_scoped_targetted_services::RegisterSimTurtle;
using io_turtle_scoped_targetted_services::TeleportTurtle;

class SimTurtle {
public:
    SimTurtle(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, std::string name, Turtle turtle, 
            float radius, float collision_range, float x_min, float x_max, float y_min, float y_max)
        :  _name(name), _turtle(turtle), _turtle_radius(radius), _collision_range(collision_range),
        _x_min(x_min), _x_max(x_max), _y_min(y_min), _y_max(y_max){

        // ROS topic queue size
        int queue_size;

        ROS_INFO_STREAM("initialize" << _turtle._cmd_vel << this);

        // Initialize parameters
        priv_nh.param("queue_size", queue_size, 100);

        // Construct ROS subscribers and publishers

        _velocity_sub = nh.subscribe(_name+"/sim/cmd_vel", queue_size, &SimTurtle::velocity_callback, this);
        _pose_pub = nh.advertise<geometry_msgs::Pose2D>(_name+"/sim/pose", queue_size);
        _sensors_pub = nh.advertise<io_turtle_scoped_targetted_msgs::DistanceSensor>(_name+"/sim/sensors", queue_size);
    }

    void update_pose(float time_step) {
        // ROS_INFO_STREAM("update_pose" << this);

        _turtle._current_vel = _turtle._cmd_vel;
        _turtle._pose.theta = std::fmod(_turtle._pose.theta + _turtle._current_vel.angular.z * time_step, 2 * M_PI);

        float delta_pose_x = std::cos(_turtle._pose.theta) * _turtle._current_vel.linear.x * time_step;
        float delta_pose_y = std::sin(_turtle._pose.theta) * _turtle._current_vel.linear.x * time_step;

        float pre_clamped_x = _turtle._pose.x + delta_pose_x;
        float pre_clamped_y = _turtle._pose.y + delta_pose_y;

        _turtle._pose.x = std::fmax(_x_min, std::fmin(_x_max, pre_clamped_x));
        _turtle._pose.y = std::fmax(_y_min, std::fmin(_y_max, pre_clamped_y));

        _pose_pub.publish(_turtle._pose);
    }

    void update_sensor(std::unordered_map<std::string, SimTurtle>& turtles) {
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
        for (const std::pair<const std::string, SimTurtle>& turtle_entry : turtles) {
            if (turtle_entry.first == _name)
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

        _sensors_pub.publish(_turtle._sensors);
    }

    bool within_boundary(float x, float y) const {
        return (x >= _x_min + _collision_range) && (x <= _x_max - _collision_range) &&
               (y >= _y_min + _collision_range) && (y <= _y_max - _collision_range);
    }

    bool teleport_turtle(TeleportTurtle::Request& req, TeleportTurtle::Response& response) {
        if (!within_boundary(req.pose.x, req.pose.y)) {
            ROS_INFO("Turtle cannot be teleported outside boundary.");
            response.data = false;
            return true;
        }

        _turtle._pose.x = req.pose.x;
        _turtle._pose.y = req.pose.y;
        _turtle._pose.theta = req.pose.theta;

        ROS_INFO("Teleported Turtle to (%f, %f, %f)", req.pose.x, req.pose.y, req.pose.theta);

        response.data = true;
        return true;
    }

    void velocity_callback(const geometry_msgs::Twist& vel) {
        _turtle._cmd_vel = vel;
    }

    void set_pose(const float x, const float y, const float theta){
        ROS_INFO_STREAM(""<<x<<" "<<y<<" "<<theta << this);
        _turtle._pose.x = x;
        _turtle._pose.y = y;
        _turtle._pose.theta = theta;
        ROS_INFO_STREAM(""<<_turtle._pose.x<< " "<<_turtle._pose.y<< " "<<_turtle._pose.theta);
        ROS_INFO_STREAM(""<<_turtle._cmd_vel);
    }

private:
        Turtle _turtle;

        std::string _name;
        ros::ServiceServer _teleport_service;

        // ROS subscribers and publishers
        ros::Subscriber _velocity_sub;
        ros::Publisher _pose_pub;
        ros::Publisher _sensors_pub;

        float _turtle_radius;
        float _collision_range;
        int _x_min;
        int _x_max;
        int _y_min;
        int _y_max;


};

using TurtleMap = std::unordered_map<std::string, SimTurtle>;
using TurtleEntry = std::pair<const std::string, SimTurtle>;

class SimEnv {
public:
    SimEnv(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, float time_step)
            : _nh(nh), _priv_nh(priv_nh), _time_step(time_step) {
        srand(time(NULL));

        // ROS topic queue size
        int queue_size;

        // Initialize parameters
        _priv_nh.param("queue_size", queue_size, 100);
        _nh.param("turtle_radius", _turtle_radius, 1.0f);
        _nh.param("collision_range", _collision_range, 2.0f);
        _nh.param("boundary/x_min", _x_min, -10);
        _nh.param("boundary/x_max", _x_max, 10);
        _nh.param("boundary/y_min", _y_min, -10);
        _nh.param("boundary/y_max", _y_max, 10);

        // Construct services
        _register_service = _nh.advertiseService("register_sim_turtle", &SimEnv::register_turtle, this);
        ROS_INFO("'Register simulation turtle' service started.");

        _teleport_service = nh.advertiseService("teleport_turtle", &SimEnv::teleport_turtle, this);
        ROS_INFO("'Teleport turtle' service started.");

    }

    void update() {
        for (TurtleEntry& turtle : _turtles) {
            turtle.second.update_pose(_time_step);
            turtle.second.update_sensor(_turtles);
        }
    }

    static float random_value(float min, float max) {
        return min + (max - min) * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    }

private:
    bool within_boundary(float x, float y) const {
        return (x >= _x_min + _collision_range) && (x <= _x_max - _collision_range) &&
               (y >= _y_min + _collision_range) && (y <= _y_max - _collision_range);
    }

    bool register_turtle(RegisterSimTurtle::Request& req, RegisterSimTurtle::Response& response) {
        if (_turtles.find(req.name) != _turtles.end()) {
            ROS_ERROR_STREAM(req.name << " already exists in the simulation environment.");
            response.data = false;
            return true;
        }
        
        Turtle new_turtle = Turtle();
        new_turtle._pose.x = random_value(_x_min + _collision_range, _x_max - _collision_range);
        new_turtle._pose.y = random_value(_y_min + _collision_range, _y_max - _collision_range);
        new_turtle._pose.theta = 0.0f;

        // SimTurtle new_simturtle(_nh, _priv_nh, req.name, new_turtle, _turtle_radius, _collision_range, _x_min, _x_max, _y_min, _y_max);

        // _turtles.emplace(req.name, new_simturtle);
        _turtles.emplace(std::piecewise_construct, 
                            std::forward_as_tuple(req.name),
                            std::forward_as_tuple(_nh, _priv_nh, req.name, new_turtle, _turtle_radius, _collision_range, _x_min, _x_max, _y_min, _y_max));

        ROS_INFO_STREAM("New turtle detected: " << req.name <<  " and initialized at (" 
            << new_turtle._pose.x << ',' << new_turtle._pose.y << ',' << new_turtle._pose.theta << ").");

        response.data = true;
        return true;
    }

    bool teleport_turtle(TeleportTurtle::Request& req, TeleportTurtle::Response& response) {
        TurtleMap::iterator turtle_it = _turtles.find(req.name);

        if (turtle_it == _turtles.end()) {
            ROS_INFO_STREAM(req.name << " is not registered in simulation.");
            response.data = false;
            return true;
        }

        if (!within_boundary(req.pose.x, req.pose.y)) {
            ROS_INFO_STREAM(req.name << " cannot be teleported outside boundary.");
            response.data = false;
            return true;
        }

        turtle_it->second.set_pose(req.pose.x, req.pose.y, req.pose.theta);

        ROS_INFO_STREAM("Teleported " << req.name << " to (" << req.pose.x << ',' << req.pose.y << ',' << req.pose.theta << ").");

        response.data = true;
        return true;
    }

    ros::NodeHandle _nh, _priv_nh;
    ros::ServiceServer _register_service;
    ros::ServiceServer _teleport_service;

    float _time_step;
    float _turtle_radius;
    float _collision_range;
    int _x_min;
    int _x_max;
    int _y_min;
    int _y_max;

    TurtleMap _turtles;
};

}  // namespace turtlesim

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "io_turtle_sim_env_node");
    ROS_INFO("Started Simulation Environment Node.");

    float time_step;

    // Load parameters
    ros::NodeHandle nh, priv_nh("~");
    priv_nh.param("time_step", time_step, 0.1f);

    // Construct simulation environment
    turtlesim::SimEnv sim_env(nh, priv_nh, time_step);

    // Set ROS loop rate
    ros::Rate rate(1.0 / time_step);

    while (ros::ok()) {
        sim_env.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
