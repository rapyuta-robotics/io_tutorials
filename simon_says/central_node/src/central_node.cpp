#include "simon_says/central_node.hpp"

namespace simon_says {

CentralNode::CentralNode(ros::NodeHandle& nh) : _ui_cmd_req_updated(false), _stale_command(false) {
    _timer = nh.createTimer(ros::Duration(1.0), &CentralNode::timer_cb, this, true, false);
    _ui_set_mode_sub = nh.subscribe("ui/set_mode", 10, &CentralNode::ui_set_mode_cb, this);
    _ui_command_req_sub = nh.subscribe("ui/command", 10, &CentralNode::ui_command_req_cb, this);
    _ui_device_status_pub = nh.advertise<StatusList>("ui/device_status", 1);

    _devices.reserve(DeviceId::NUM_DEVICES);

    _devices.emplace_back(nh, "joystick");
    _devices.emplace_back(nh, "turtlebot");
    _devices.emplace_back(nh, "alphabot");
    _devices.emplace_back(nh, "gopigo");
    _devices.emplace_back(nh, "quad");

    _devices[DeviceId::JOYSTICK].set_scaling(1, 1);

    _devices[DeviceId::TURTLEBOT].set_scaling(1, 1);
    _devices[DeviceId::TURTLEBOT].set_bounds(0.6, 0.99);

    _devices[DeviceId::ALPHABOT].set_scaling(0.5, 1);
    _devices[DeviceId::ALPHABOT].set_bounds(0.3, 0.99);

    _devices[DeviceId::GOPIGO].set_scaling(1, 1);
    _devices[DeviceId::GOPIGO].set_bounds(0.6, 0.99);

    _devices[DeviceId::QUAD].set_scaling(1, 1);
    _devices[DeviceId::QUAD].set_bounds(0.6, 0.99);
}

void CentralNode::update() {
    ui_publish_device_status();
    publish_mode_req_to_device();
    publish_command_to_devices();
}

void CentralNode::timer_cb(const ros::TimerEvent& event) {
    _stale_command = true;
}

void CentralNode::ui_set_mode_cb(const SetMode& set_mode) {
    _devices[set_mode.device.id].set_mode(set_mode.mode.data);
}

void CentralNode::ui_command_req_cb(const geometry_msgs::Twist& cmd) {
    _ui_cmd_req = cmd;
    _ui_cmd_req.linear.x *= 0.05;
    _ui_cmd_req.angular.z *= 0.33;
    _ui_cmd_req_updated = true;
}

void CentralNode::ui_publish_device_status() const {
    StatusList device_status_list;
    for (const Device& device : _devices) {
        device_status_list.status_list.push_back(device.status());
    }
    _ui_device_status_pub.publish(device_status_list);
}

void CentralNode::publish_mode_req_to_device() const {
    for (const Device& device : _devices) {
        device.publish_mode_req();
    }
}

void CentralNode::publish_command_to_devices() {
    bool device_source = false;
    for (Device& device : _devices) {
        // Choose the first source
        if (device.mode().data & Mode::SOURCE) {
            device_source = true;
            _cmd_vel = device.get_cmd_req();
            break;
        }
    }
    if (device_source || _ui_cmd_req_updated) {
        _ui_cmd_req_updated = false;
        _stale_command = false;
        _timer.stop();
        _timer.start();
        if (!device_source) {
            _cmd_vel = _ui_cmd_req;
        }
    }
    if (!_stale_command) {
        for (const Device& device : _devices) {
            device.publish_command(_cmd_vel);
        }
    }
}

}  // namespace simon_says

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "central_node");

    double freq;

    // Load parameters
    ros::NodeHandle nh, priv_nh("~");
    priv_nh.param("freq", freq, 20.0);

    // Construct central node
    simon_says::CentralNode node(nh);

    // Set ROS loop rate
    ros::Rate rate(freq);

    while (ros::ok()) {
        node.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
