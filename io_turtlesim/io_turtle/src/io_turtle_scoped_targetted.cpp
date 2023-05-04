#include <std_msgs/String.h>

#include "io_turtle/io_turtle.hpp"
#include "io_turtle/io_turtle_scoped_targetted.hpp"

std::string rapyuta_io_peers;
void rapyuta_io_peer_callback(const std_msgs::String& msg){
    rapyuta_io_peers = msg.data;
}

std::vector<std::string> split(const std::string input, const char delim){
    std::vector<std::string> output;
    std::stringstream ss{input};
    std::string buf;
    while (std::getline(ss, buf, delim)) {
      output.push_back(buf);
    }
    return output;
}

namespace turtlesim {

IOTurtleScopedTargetted::IOTurtleScopedTargetted(ros::NodeHandle& priv_nh) : IOTurtleBase<geometry_msgs::Twist, geometry_msgs::Pose2D>(priv_nh)
{
    _turtle = new TurtleGeometryMsg();
}

IOTurtleScopedTargetted::~IOTurtleScopedTargetted()
{
    delete _turtle;
}

IOTurtleScopedTargettedNode::IOTurtleScopedTargettedNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh) : IOTurtleBaseNode<geometry_msgs::Twist, geometry_msgs::Pose2D>(nh, priv_nh) {
    _nh.param("use_rapyuta_io", _use_rapyuta_io, false);
    priv_nh.param<std::string>("name", _name, "turtle0");
    _io_turtle = new IOTurtleScopedTargetted(priv_nh);
}

IOTurtleScopedTargettedNode::~IOTurtleScopedTargettedNode(){
    delete _io_turtle;
}

void IOTurtleScopedTargettedNode::set_dr(){
    // Set up parameter dynamic reconfiguration
    dynamic_reconfigure::Server<io_turtle::ControlConfig> dr_srv;
    dynamic_reconfigure::Server<io_turtle::ControlConfig>::CallbackType dr_cb;
    dr_cb = boost::bind(&IOTurtleScopedTargetted::dynamic_parameters_callback, _io_turtle, _1, _2);
    dr_srv.setCallback(dr_cb);
}

void IOTurtleScopedTargettedNode::register_turtle(){
    const std::string peer_topic_name = "/rapyuta_io_peers";
    ros::Subscriber rapyuta_io_peer_sub = _nh.subscribe(peer_topic_name, 1, rapyuta_io_peer_callback);
    if(_use_rapyuta_io){
        ros::topic::waitForMessage<std_msgs::String>(peer_topic_name, _nh);
        ros::spinOnce();
        _name = split(rapyuta_io_peers, ',')[0];
    }

    if (_name.rfind("turtle", 0) == 0) {
        _id = _name.back() - '0';
    }
    else{
        ROS_ERROR_STREAM("Turtle deployment ROS Environment alias must start weiht turtle");
        return ;
    }

    _io_turtle->register_turtle(_nh, _id, _sim, _queue_size);
    ROS_INFO("Registered myself as Turtle%d", _id);
}

}  // namespace turtlesim
