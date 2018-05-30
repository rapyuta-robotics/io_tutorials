#ifndef SIMON_SAYS_CENTRAL_NODE_HPP
#define SIMON_SAYS_CENTRAL_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <simon_says/DeviceId.h>
#include <simon_says/SetMode.h>
#include <simon_says/StatusList.h>

#include <simon_says/device.hpp>

namespace simon_says {

class CentralNode {
public:
    CentralNode(ros::NodeHandle& nh);
    void update();

private:
    void timer_cb(const ros::TimerEvent& event);
    void ui_set_mode_cb(const SetMode& set_mode);
    void ui_command_req_cb(const geometry_msgs::Twist& cmd);
    void ui_publish_device_status() const;
    void publish_mode_req_to_device() const;
    void publish_command_to_devices();

    ros::Subscriber _ui_set_mode_sub;
    ros::Subscriber _ui_command_req_sub;
    ros::Publisher _ui_device_status_pub;

    geometry_msgs::Twist _ui_cmd_req;
    geometry_msgs::Twist _cmd_vel;

    ros::Timer _timer;
    std::vector<Device> _devices;
    bool _ui_cmd_req_updated;
    bool _stale_command;
};

}  // namespace simon_says

#endif /* SIMON_SAYS_CENTRAL_NODE_HPP */
