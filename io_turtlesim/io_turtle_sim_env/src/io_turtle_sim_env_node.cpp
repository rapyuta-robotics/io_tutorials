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

using TurtleMap = std::unordered_map<int, Turtle>;
using TurtleEntry = std::pair<const int, Turtle>;

using io_turtle_services::RegisterSimTurtle;
using io_turtle_services::TeleportTurtle;

class SimEnv {
public:
    SimEnv(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, float time_step)
            : _time_step(time_step) {
        srand(time(NULL));

        // ROS topic queue size
        int queue_size;

        // Initialize parameters
        priv_nh.param("queue_size", queue_size, 100);
        nh.param("turtle_radius", _turtle_radius, 1.0f);
        nh.param("collision_range", _collision_range, 2.0f);
        nh.param("boundary/x_min", _x_min, -10);
        nh.param("boundary/x_max", _x_max, 10);
        nh.param("boundary/y_min", _y_min, -10);
        nh.param("boundary/y_max", _y_max, 10);

        // Construct services
        _register_service = nh.advertiseService("register_sim_turtle", &SimEnv::register_turtle, this);
        ROS_INFO("'Register simulation turtle' service started.");

        _teleport_service = nh.advertiseService("teleport_turtle", &SimEnv::teleport_turtle, this);
        ROS_INFO("'Teleport turtle' service started.");

        // Construct ROS subscribers and publishers
        _velocity_sub = nh.subscribe("sim/cmd_vel", queue_size, &SimEnv::velocity_callback, this);
        _pose_pub = nh.advertise<io_turtle_msgs::Pose>("sim/pose", queue_size);
        _sensors_pub = nh.advertise<io_turtle_msgs::DistanceSensor>("sim/sensors", queue_size);
    }

    void update() {
        for (TurtleEntry& turtle : _turtles) {
            update_pose(turtle.second, _time_step);
            update_sensor(turtle.second);
            _pose_pub.publish(turtle.second._pose);
            _sensors_pub.publish(turtle.second._sensors);
        }
    }

    void update_sensor(Turtle& turtle) {
        float cos_theta = std::cos(turtle._pose.theta);
        float sin_theta = std::sin(turtle._pose.theta);

        // Handle boundary distances
        float boundary_distance[4];
        boundary_distance[0] = (cos_theta == 0.0f) ? 0.0f : (_x_min - turtle._pose.x) / cos_theta;
        boundary_distance[1] = (cos_theta == 0.0f) ? 0.0f : (_x_max - turtle._pose.x) / cos_theta;
        boundary_distance[2] = (sin_theta == 0.0f) ? 0.0f : (_y_min - turtle._pose.y) / sin_theta;
        boundary_distance[3] = (sin_theta == 0.0f) ? 0.0f : (_y_max - turtle._pose.y) / sin_theta;

        std::sort(boundary_distance, boundary_distance + 4);
        turtle._sensors.rear = boundary_distance[1];
        turtle._sensors.front = boundary_distance[2];

        // Handle obstacle distances
        for (const TurtleEntry& turtle_entry : _turtles) {
            if (turtle_entry.first == turtle._id)
                continue;

            // Transform coordinates of the turtles to current turtle axis
            float dx = turtle_entry.second._pose.x - turtle._pose.x;
            float dy = turtle_entry.second._pose.y - turtle._pose.y;
            float tx = dx * cos_theta + dy * sin_theta;
            float ty = -dx * sin_theta + dy * cos_theta;

            // Only consider turtles within collision range
            if (std::fabs(ty) >= _turtle_radius)
                continue;

            // Find the distance to closest obstacle and its position w.r.t current turtle
            float intersection = std::sqrt(_turtle_radius * _turtle_radius - ty * ty);
            if (tx > 0) {
                float distance = std::fmin(tx + intersection, tx - intersection);
                if (distance < turtle._sensors.front) {
                    turtle._sensors.front = distance;
                }
            } else if (tx < 0) {
                float distance = std::fmax(tx + intersection, tx - intersection);
                if (distance > turtle._sensors.rear) {
                    turtle._sensors.rear = distance;
                }
            }
        }

        turtle._sensors.rear = std::fabs(turtle._sensors.rear);
    }

    void update_pose(Turtle& turtle, float time_step) {
        turtle._pose.linear_velocity = turtle._cmd_vel.linear_velocity;
        turtle._pose.angular_velocity = turtle._cmd_vel.angular_velocity;
        turtle._pose.theta = std::fmod(turtle._pose.theta + turtle._pose.angular_velocity * time_step, 2 * M_PI);

        float delta_pose_x = std::cos(turtle._pose.theta) * turtle._pose.linear_velocity * time_step;
        float delta_pose_y = std::sin(turtle._pose.theta) * turtle._pose.linear_velocity * time_step;

        float pre_clamped_x = turtle._pose.x + delta_pose_x;
        float pre_clamped_y = turtle._pose.y + delta_pose_y;

        turtle._pose.x = std::fmax(_x_min, std::fmin(_x_max, pre_clamped_x));
        turtle._pose.y = std::fmax(_y_min, std::fmin(_y_max, pre_clamped_y));
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
        if (_turtles.find(req.id) != _turtles.end()) {
            ROS_ERROR("Turtle%d already exists in the simulation environment.", req.id);
            response.data = false;
            return true;
        }

        Turtle new_turtle(req.id);
        new_turtle._pose.x = random_value(_x_min + _collision_range, _x_max - _collision_range);
        new_turtle._pose.y = random_value(_y_min + _collision_range, _y_max - _collision_range);
        new_turtle._pose.theta = 0.0f;

        _turtles.emplace(new_turtle._id, new_turtle);

        ROS_INFO("New turtle detected: Turtle%d and initialized at (%f, %f, %f).", req.id, new_turtle._pose.x,
                new_turtle._pose.y, new_turtle._pose.theta);

        response.data = true;
        return true;
    }

    bool teleport_turtle(TeleportTurtle::Request& req, TeleportTurtle::Response& response) {
        TurtleMap::iterator turtle_it = _turtles.find(req.id);

        if (turtle_it == _turtles.end()) {
            ROS_INFO("Turtle%d is not registered in simulation.", req.id);
            response.data = false;
            return true;
        }

        if (!within_boundary(req.x, req.y)) {
            ROS_INFO("Turtle%d cannot be teleported outside boundary.", req.id);
            response.data = false;
            return true;
        }

        turtle_it->second._pose.x = req.x;
        turtle_it->second._pose.y = req.y;
        turtle_it->second._pose.theta = req.theta;

        ROS_INFO("Teleported Turtle%d to (%f, %f, %f)", req.id, req.x, req.y, req.theta);

        response.data = true;
        return true;
    }

    void velocity_callback(const io_turtle_msgs::Velocity& vel) {
        TurtleMap::iterator turtle_it = _turtles.find(vel.id);
        if (turtle_it == _turtles.end()) {
            ROS_ERROR("Turtle%d not registered in simulation.", vel.id);
        } else {
            turtle_it->second._cmd_vel = vel;
        }
    }

    ros::ServiceServer _register_service;
    ros::ServiceServer _teleport_service;
    ros::Subscriber _velocity_sub;
    ros::Publisher _pose_pub;
    ros::Publisher _sensors_pub;

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
