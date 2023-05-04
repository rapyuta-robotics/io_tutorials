#include "io_turtle/io_turtle.hpp"

namespace turtlesim {

IOTurtle::IOTurtle(ros::NodeHandle& priv_nh) : IOTurtleBase<io_turtle_msgs::Velocity, io_turtle_msgs::Pose>(priv_nh)
{
    _turtle = new TurtleCustomMsg();
}

IOTurtle::~IOTurtle()
{
    delete _turtle;
}

void IOTurtle::register_turtle(ros::NodeHandle& nh, int id, bool sim, int queue_size) {

    IOTurtleBase<io_turtle_msgs::Velocity, io_turtle_msgs::Pose>::register_turtle(nh, id, sim, queue_size);
    _turtle->set_id(id);
    if (sim) {
        _sim_pose_sub = nh.subscribe("sim/pose", queue_size, &IOTurtle::sim_pose_callback, this);
        _sim_sensors_sub = nh.subscribe("sim/sensors", queue_size, &IOTurtle::sim_sensors_callback, this);
    }

    _velocity_sub = nh.subscribe("cmd_vel", queue_size, &IOTurtle::velocity_callback, this);
}

void IOTurtle::velocity_callback(const io_turtle_msgs::Velocity& vel) {
    if (vel.id != _turtle->_id) {
        return;
    }

    if ((std::fabs(vel.linear_velocity) < 1e-4f && std::fabs(vel.angular_velocity) < 1e-4f)) {
        _stop_requested = true;
    }

    IOTurtleBase<io_turtle_msgs::Velocity, io_turtle_msgs::Pose>::velocity_callback(vel);
}

void IOTurtle::sim_pose_callback(const io_turtle_msgs::Pose& pose) {
    if (pose.id == _turtle->_id) {
        IOTurtleBase<io_turtle_msgs::Velocity, io_turtle_msgs::Pose>::sim_pose_callback(pose);
    }
}

void IOTurtle::sim_sensors_callback(const io_turtle_msgs::DistanceSensor& sensors) {
    if (sensors.id == _turtle->_id) {
        IOTurtleBase<io_turtle_msgs::Velocity, io_turtle_msgs::Pose>::sim_sensors_callback(sensors);
    }
}

IOTurtleNode::IOTurtleNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh) : IOTurtleBaseNode<io_turtle_msgs::Velocity, io_turtle_msgs::Pose>(nh, priv_nh) {
    // _nh.param("use_rapyuta_io", use_rapyuta_io, false);
    _io_turtle = new IOTurtle(priv_nh);
}

IOTurtleNode::~IOTurtleNode(){
    delete _io_turtle;
}

void IOTurtleNode::set_dr(){
    // Set up parameter dynamic reconfiguration
    dynamic_reconfigure::Server<io_turtle::ControlConfig> dr_srv;
    dynamic_reconfigure::Server<io_turtle::ControlConfig>::CallbackType dr_cb;
    dr_cb = boost::bind(&IOTurtle::dynamic_parameters_callback, _io_turtle, _1, _2);
    dr_srv.setCallback(dr_cb);
}

void IOTurtleNode::register_turtle(){

    // Construct ROS service for registration with command center
    ros::ServiceClient client = _nh.serviceClient<RegisterTurtle>("register_turtle");
    client.waitForExistence();

    // Service request
    RegisterTurtle req;

    // Send request
    if (!client.call(req)) {
        ROS_ERROR_STREAM("Unable to make a call to register turtle at the command center.");
        return ;
    }
    // Successfully register with command center
    _id = req.response.id;

    IOTurtleBaseNode<io_turtle_msgs::Velocity, io_turtle_msgs::Pose>::register_turtle();
    // _io_turtle->register_turtle(_nh, _id, _sim, _queue_size);
    ROS_INFO("Registered myself as Turtle%d", _id);
}

void IOTurtleNode::set_action_name(){
    _action_server_name = "turtle" + std::to_string(_id) + "/goto_action";
}

}  // namespace turtlesim
