#include "io_turtle/io_turtle.hpp"

namespace turtlesim {

static float clamp_magnitude(float x, float max) {
    return std::fabs(x) > max ? std::copysign(max, x) : x;
}

IOTurtle::IOTurtle(ros::NodeHandle& priv_nh)
        : _sim(true)
        , _reached_goal(false)
        , _stop_requested(false)
        , _goal_x(0.0f)
        , _goal_y(0.0f) {
    priv_nh.param("obstacle_field_range", _d_0, 6.0f);
    priv_nh.param("collision_range", _collision_range, 2.0f);
    priv_nh.param("goal_tolerance", _goal_tolerance, 0.5f);
}

void IOTurtle::dynamic_parameters_callback(io_turtle::ControlConfig& config, uint32_t level) {
    _control_config = config;
}

void IOTurtle::register_turtle(ros::NodeHandle& nh, int id, bool sim, int queue_size) {
    _turtle.set_id(id);
    _sim = sim;

    if (_sim) {
        _sim_pose_sub = nh.subscribe("sim/pose", queue_size, &IOTurtle::sim_pose_callback, this);
        _sim_sensors_sub = nh.subscribe("sim/sensors", queue_size, &IOTurtle::sim_sensors_callback, this);
        _sim_velocity_pub = nh.advertise<io_turtle_msgs::Velocity>("sim/cmd_vel", 1);
    }

    _velocity_sub = nh.subscribe("cmd_vel", queue_size, &IOTurtle::velocity_callback, this);
    _pose_pub = nh.advertise<io_turtle_msgs::Pose>("pose", 1);
}

void IOTurtle::set_goal(float x, float y) {
    ROS_INFO("Request for Turtle%d to move to (%f, %f) is received.", _turtle._id, x, y);
    _goal_x = x;
    _goal_y = y;
}

void IOTurtle::move_towards_goal() {
    // Transform goal position into coordinates of turtle body frame
    float cos_theta = std::cos(_turtle._pose.theta);
    float sin_theta = std::sin(_turtle._pose.theta);
    float dx = _goal_x - _turtle._pose.x;
    float dy = _goal_y - _turtle._pose.y;
    float tx_g = dx * cos_theta + dy * sin_theta;
    float ty_g = -dx * sin_theta + dy * cos_theta;

    // Calculate goal distance in body frame of turtle
    float d_g = tx_g * tx_g + ty_g * ty_g;
    if (d_g <= _goal_tolerance * _goal_tolerance) {
        _reached_goal = true;
        return;
    }

    float F_att_x = _control_config.k_att * tx_g;
    float F_att_y = _control_config.k_att * ty_g;

    // Circular cropping of attraction force
    float F_mag = std::sqrt(F_att_x * F_att_x + F_att_y * F_att_y);
    if (F_mag > _control_config.f_att_max) {
        const float force_reduction = _control_config.f_att_max / F_mag;
        F_att_x *= force_reduction;
        F_att_y *= force_reduction;
    }

    // Repulsive force is measured using the sensor information
    float d_obs = 0.0, F_rep_x = 0.0;
    int obs_direction = 1;

    if (_turtle._pose.linear_velocity >= 0.0) {
        d_obs = _turtle._sensors.front;
    } else {
        d_obs = _turtle._sensors.rear;
        obs_direction = -1;
    }

    if (d_obs <= _d_0 && d_obs >= 1e-2f) {
        F_rep_x = obs_direction * _control_config.k_rep * ((d_obs - _d_0) / std::pow(d_obs, 3));
    }

    float linear = clamp_magnitude(F_att_x + F_rep_x, _control_config.v_max);
    float heading_error = std::atan2(F_att_y, linear);
    float angular = clamp_magnitude(_control_config.k_theta * heading_error, _control_config.omega_max);

    _turtle.set_cmd_vel(linear, angular);
    publish_velocity();
}

void IOTurtle::stop_turtle() {
    _turtle.set_cmd_vel(0.0, 0.0);
    publish_velocity();
}

void IOTurtle::reset() {
    _stop_requested = false;
    _reached_goal = false;
}

bool IOTurtle::is_moving() const {
    return std::fabs(_turtle._pose.linear_velocity) > 1e-4f || std::fabs(_turtle._pose.angular_velocity) > 1e-4f;
}

bool IOTurtle::stop_requested() const {
    return _stop_requested;
}

bool IOTurtle::reached_goal() const {
    return _reached_goal;
}

bool IOTurtle::check_collision() const {
    // check if the turtle is in collision range in its direction of motion
    return ((_turtle._pose.linear_velocity > 0.0 && _turtle._sensors.front < _collision_range) ||
            (_turtle._pose.linear_velocity < 0.0 && _turtle._sensors.rear < _collision_range));
}

void IOTurtle::publish_velocity() {
    if (_sim) {
        _sim_velocity_pub.publish(_turtle._cmd_vel);
    }
}

void IOTurtle::velocity_callback(const io_turtle_msgs::Velocity& vel) {
    if (vel.id != _turtle._id) {
        return;
    }

    if ((std::fabs(vel.linear_velocity) < 1e-4f && std::fabs(vel.angular_velocity) < 1e-4f)) {
        _stop_requested = true;
    }

    _turtle._cmd_vel = vel;
    publish_velocity();
}

void IOTurtle::sim_pose_callback(const io_turtle_msgs::Pose& pose) {
    if (pose.id == _turtle._id) {
        _turtle._pose = pose;
        _pose_pub.publish(_turtle._pose);
    }
}

void IOTurtle::sim_sensors_callback(const io_turtle_msgs::DistanceSensor& sensors) {
    if (sensors.id == _turtle._id) {
        _turtle._sensors = sensors;
    }
}

}  // namespace turtlesim
