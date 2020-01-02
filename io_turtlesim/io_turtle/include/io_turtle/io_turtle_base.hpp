#ifndef IO_TURTLE_IO_TURTLE_BASE_HPP
#define IO_TURTLE_IO_TURTLE_BASE_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <io_turtle/turtle.hpp>
#include <io_turtle/ControlConfig.h>
#include <io_turtle_services/RegisterTurtle.h>
#include <io_turtle_services/RegisterSimTurtle.h>
#include <io_turtle_action/GoToAction.h>

using namespace io_turtle_services;
using namespace io_turtle_action;

namespace turtlesim {

template<class VelMsg, class PoseMsg>
class IOTurtleBase {
public:
    IOTurtleBase(ros::NodeHandle& priv_nh);

    void dynamic_parameters_callback(io_turtle::ControlConfig& config, uint32_t level);
    virtual void register_turtle(ros::NodeHandle& nh, int id, bool sim, int queue_size);
    void set_goal(float x, float y);
    void move_towards_goal();
    void stop_turtle();
    void reset();

    bool is_moving() const;
    bool stop_requested() const;
    bool reached_goal() const;
    bool check_collision() const;

protected:
    void publish_velocity();
    void velocity_callback(const VelMsg& vel);
    void sim_pose_callback(const PoseMsg& pose);
    void sim_sensors_callback(const io_turtle_msgs::DistanceSensor& sensor);

    ros::Subscriber _velocity_sub;
    ros::Subscriber _sim_pose_sub;
    ros::Subscriber _sim_sensors_sub;
    ros::Publisher _sim_velocity_pub;
    ros::Publisher _pose_pub;

    TurtleBase<VelMsg, PoseMsg>* _turtle;
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

template<class VelMsg, class PoseMsg>
class GoToActionServer {
public:
    GoToActionServer(ros::NodeHandle& nh, const std::string& server_name, turtlesim::IOTurtleBase<VelMsg, PoseMsg>* io_turtle,
            float frequency, float timeout);

    void timer_callback(const ros::TimerEvent& event);
    void goto_action_callback(const GoToGoalConstPtr& goal);

private:
    turtlesim::IOTurtleBase<VelMsg, PoseMsg>* _io_turtle;
    actionlib::SimpleActionServer<GoToAction> _action_server;
    ros::Timer _timer;
    GoToFeedback _feedback;
    GoToResult _result;
    float _frequency;
    bool _preempted;
};

template<class VelMsg, class PoseMsg>
class IOTurtleBaseNode {
public:
    IOTurtleBaseNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    virtual void register_turtle();
    virtual void register_sim_turtle();
    virtual void set_dr();
    void spin();
protected:
    virtual void set_action_name();
    ros::NodeHandle _nh;
    IOTurtleBase<VelMsg, PoseMsg>* _io_turtle;
    int _id;
    bool _sim;
    int _queue_size;
    float _frequency;
    float _timeout;
    std::string _action_server_name;
};

static float clamp_magnitude(float x, float max) {
    return std::fabs(x) > max ? std::copysign(max, x) : x;
}

template<class VelMsg, class PoseMsg>
IOTurtleBase<VelMsg, PoseMsg>::IOTurtleBase(ros::NodeHandle& priv_nh)
        : _sim(true)
        , _reached_goal(false)
        , _stop_requested(false)
        , _goal_x(0.0f)
        , _goal_y(0.0f) {
    priv_nh.param("obstacle_field_range", _d_0, 6.0f);
    priv_nh.param("collision_range", _collision_range, 2.0f);
    priv_nh.param("goal_tolerance", _goal_tolerance, 0.5f);
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::dynamic_parameters_callback(io_turtle::ControlConfig& config, uint32_t level) {
    _control_config = config;
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::register_turtle(ros::NodeHandle& nh, int id, bool sim, int queue_size) {
    _turtle->set_id(id);
    _sim = sim;

    if (_sim) {
        _sim_pose_sub = nh.subscribe("sim/pose", queue_size, &IOTurtleBase<VelMsg, PoseMsg>::sim_pose_callback, this);
        _sim_sensors_sub = nh.subscribe("sim/sensors", queue_size, &IOTurtleBase<VelMsg, PoseMsg>::sim_sensors_callback, this);
        _sim_velocity_pub = nh.advertise<VelMsg>("sim/cmd_vel", 1);
    }

    _velocity_sub = nh.subscribe("cmd_vel", queue_size, &IOTurtleBase<VelMsg, PoseMsg>::velocity_callback, this);
    _pose_pub = nh.advertise<PoseMsg>("pose", 1);
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::set_goal(float x, float y) {
    ROS_INFO("Request for Turtle%d to move to (%f, %f) is received.", _turtle->_id, x, y);
    _goal_x = x;
    _goal_y = y;
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::move_towards_goal() {
    // Transform goal position into coordinates of turtle body frame
    float cos_theta = std::cos(_turtle->_pose.theta);
    float sin_theta = std::sin(_turtle->_pose.theta);
    float dx = _goal_x - _turtle->_pose.x;
    float dy = _goal_y - _turtle->_pose.y;
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

    if (_turtle->get_linear_velocity() >= 0.0) {
        d_obs = _turtle->_sensors.front;
    } else {
        d_obs = _turtle->_sensors.rear;
        obs_direction = -1;
    }

    if (d_obs <= _d_0 && d_obs >= 1e-2f) {
        F_rep_x = obs_direction * _control_config.k_rep * ((d_obs - _d_0) / std::pow(d_obs, 3));
    }

    float linear = clamp_magnitude(F_att_x + F_rep_x, _control_config.v_max);
    float heading_error = std::atan2(F_att_y, linear);
    float angular = clamp_magnitude(_control_config.k_theta * heading_error, _control_config.omega_max);

    _turtle->set_cmd_vel(linear, angular);
    publish_velocity();
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::stop_turtle() {
    _turtle->set_cmd_vel(0.0, 0.0);
    publish_velocity();
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::reset() {
    _stop_requested = false;
    _reached_goal = false;
}

template<class VelMsg, class PoseMsg>
bool IOTurtleBase<VelMsg, PoseMsg>::is_moving() const {
    return std::fabs(_turtle->get_linear_velocity()) > 1e-4f || std::fabs(_turtle->get_angular_velocity()) > 1e-4f;
}

template<class VelMsg, class PoseMsg>
bool IOTurtleBase<VelMsg, PoseMsg>::stop_requested() const {
    return _stop_requested;
}

template<class VelMsg, class PoseMsg>
bool IOTurtleBase<VelMsg, PoseMsg>::reached_goal() const {
    return _reached_goal;
}

template<class VelMsg, class PoseMsg>
bool IOTurtleBase<VelMsg, PoseMsg>::check_collision() const {
    // check if the turtle is in collision range in its direction of motion
    return ((_turtle->get_linear_velocity() > 0.0 && _turtle->_sensors.front < _collision_range) ||
            (_turtle->get_linear_velocity() < 0.0 && _turtle->_sensors.rear < _collision_range));
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::publish_velocity() {
    if (_sim) {
        _sim_velocity_pub.publish(_turtle->_cmd_vel);
    }
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::velocity_callback(const VelMsg& vel) {
    _turtle->_cmd_vel = vel;
    publish_velocity();
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::sim_pose_callback(const PoseMsg& pose) {
    _turtle->_pose = pose;
    _pose_pub.publish(_turtle->_pose);
}

template<class VelMsg, class PoseMsg>
void IOTurtleBase<VelMsg, PoseMsg>::sim_sensors_callback(const io_turtle_msgs::DistanceSensor& sensors) {
    _turtle->_sensors = sensors;
}

template<class VelMsg, class PoseMsg>
GoToActionServer<VelMsg, PoseMsg>::GoToActionServer(ros::NodeHandle& nh, const std::string& server_name, turtlesim::IOTurtleBase<VelMsg, PoseMsg>* io_turtle,
        float frequency, float timeout)
        : _action_server(nh, server_name, boost::bind(&GoToActionServer::goto_action_callback, this, _1), false)
        , _io_turtle(io_turtle)
        , _frequency(frequency)
        , _preempted(false)
        , _timer(nh.createTimer(ros::Duration(timeout), &GoToActionServer::timer_callback, this, true, false)) {
    _action_server.start();
}

template<class VelMsg, class PoseMsg>
void GoToActionServer<VelMsg, PoseMsg>::timer_callback(const ros::TimerEvent& event) {
    ROS_INFO("Unable to reach goal.");
    _preempted = true;
}

template<class VelMsg, class PoseMsg>
void GoToActionServer<VelMsg, PoseMsg>::goto_action_callback(const GoToGoalConstPtr& goal) {
    ros::Rate rate(_frequency);

    _io_turtle->reset();
    _io_turtle->set_goal(goal->x, goal->y);

    while (!_io_turtle->reached_goal()) {
        _io_turtle->move_towards_goal();

        if (!_io_turtle->is_moving() && !_timer.hasPending()) {
            _timer.start();
        } else if (_io_turtle->is_moving() && _timer.hasPending()) {
            _timer.stop();
        }

        // Handle cases where a turtle has been stopped or a goal has been cancelled
        if (_preempted || _action_server.isPreemptRequested() || !ros::ok() || _io_turtle->stop_requested()) {
            ROS_INFO("Goal request cancelled");

            _io_turtle->stop_turtle();
            _action_server.setPreempted();
            _feedback.data = false;
            _preempted = false;
            break;
        }

        _feedback.data = true;
        _action_server.publishFeedback(_feedback);

        rate.sleep();
    }

    // Arrived at goal
    if (_feedback.data) {
        _result.data = true;
        _io_turtle->stop_turtle();
        _action_server.setSucceeded(_result);
    }

    _timer.stop();
}

template<class VelMsg, class PoseMsg>
IOTurtleBaseNode<VelMsg, PoseMsg>::IOTurtleBaseNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh):_nh(nh){

    // Load parameters
    priv_nh.param("id", _id, 0);
    priv_nh.param("sim", _sim, true);
    priv_nh.param("queue_size", _queue_size, 100);
    float time_step = 0.1f;
    priv_nh.param("time_step", time_step, 0.1f);
    _frequency = 1.0f / time_step;
    priv_nh.param("goal_timeout", _timeout, 5.0f);

}

template<class VelMsg, class PoseMsg>
void IOTurtleBaseNode<VelMsg, PoseMsg>::register_turtle(){
    _io_turtle->register_turtle(_nh, _id, _sim, _queue_size);
}

template<class VelMsg, class PoseMsg>
void IOTurtleBaseNode<VelMsg, PoseMsg>::set_action_name(){
    _action_server_name = "/goto_action";
}

template<class VelMsg, class PoseMsg>
void IOTurtleBaseNode<VelMsg, PoseMsg>::register_sim_turtle(){
    ROS_INFO("Starting node in simulation mode.");

    // Construct ROS service for registration with simulation environment
    ros::ServiceClient register_sim_client = _nh.serviceClient<RegisterSimTurtle>("register_sim_turtle");
    register_sim_client.waitForExistence();

    // Service request
    RegisterSimTurtle sim_req;
    sim_req.request.id = _id;

    // Send request
    if (!register_sim_client.call(sim_req)) {
        ROS_ERROR("Unable to make a call to register sim turtle service.");
    }

    // Validate response
    if (!sim_req.response.data) {
        ROS_ERROR("Unable to register with simulation environment.");
    }
}

template<class VelMsg, class PoseMsg>
void IOTurtleBaseNode<VelMsg, PoseMsg>::set_dr(){
    // Set up parameter dynamic reconfiguration
    dynamic_reconfigure::Server<io_turtle::ControlConfig> dr_srv;
    dynamic_reconfigure::Server<io_turtle::ControlConfig>::CallbackType dr_cb;
    dr_cb = boost::bind(&IOTurtleBase<VelMsg, PoseMsg>::dynamic_parameters_callback, _io_turtle, _1, _2);
    dr_srv.setCallback(dr_cb);
}

template<class VelMsg, class PoseMsg>
void IOTurtleBaseNode<VelMsg, PoseMsg>::spin(){

    // Set ROS loop rate
    ros::Rate rate(_frequency);

    this->register_turtle();
    if(_sim){
        register_sim_turtle();
    }

    this->set_dr();

    // Construct ROS action server for goals
    this->set_action_name();
    turtlesim::GoToActionServer<VelMsg, PoseMsg> goto_server(_nh, _action_server_name, _io_turtle, _frequency, _timeout);
    ROS_INFO("ROS action server Turtle%d is started.", _id);


    while (ros::ok()) {
        if (_io_turtle->check_collision()) {
            _io_turtle->stop_turtle();
        }
        ros::spinOnce();
        rate.sleep();
    }

}

}  // namespace turtlesim

#endif /* IO_TURTLE_IO_TURTLE_HPP */
