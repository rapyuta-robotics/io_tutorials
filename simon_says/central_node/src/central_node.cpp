#include "central_node.hpp"

namespace {
    int queue_size;
    float central_timeout;
    float ui_scaling[2];
    float joystick_scaling[2];
    float turtlebot_scaling[2];
    float turtlebot_bounds[2];
    float alphabot_scaling[2];
    float alphabot_bounds[2];
    float gopigo_scaling[2];
    float gopigo_bounds[2];
    float quad_scaling[2];
    float quad_bounds[2];
}

namespace simon_says {

CentralNode::CentralNode(ros::NodeHandle& nh) : _ui_cmd_req_updated(false), _stale_command(false) {
    nh.param("queue_size",                  queue_size,           1);
    nh.param("central_timeout",             central_timeout,      1.0f);

    _timer = nh.createTimer(ros::Duration(central_timeout), &CentralNode::timer_cb, this, true, false);
    _ui_set_mode_sub = nh.subscribe("/ui/set_mode", queue_size, &CentralNode::ui_set_mode_cb, this);
    _ui_command_req_sub = nh.subscribe("/ui/command", queue_size, &CentralNode::ui_command_req_cb, this);
    _ui_device_status_pub = nh.advertise<StatusList>("/ui/device_status", queue_size);

    _devices.reserve(DeviceId::NUM_DEVICES);
    _devices.emplace_back(nh, "joystick");
    _devices.emplace_back(nh, "turtlebot");
    _devices.emplace_back(nh, "alphabot");
    _devices.emplace_back(nh, "gopigo");
    _devices.emplace_back(nh, "quad");

    _device_status_list.status_list.reserve(DeviceId::NUM_DEVICES);
    for (int i = 0; i < DeviceId::NUM_DEVICES; ++i) {
      _device_status_list.status_list.emplace_back(Status()); 
    }

    for (int i = 0; i < DeviceId::NUM_DEVICES; ++i) {
      ROS_INFO("%d", i);
    }

    nh.param("ui_command/scaling/linear",   ui_scaling[0],        0.05f);
    nh.param("ui_command/scaling/angular",  ui_scaling[1],        0.33f);
    nh.param("joystick/scaling/linear",     joystick_scaling[0],  1.0f);
    nh.param("joystick/scaling/angular",    joystick_scaling[1],  1.0f);
    nh.param("turtlebot/scaling/linear",    turtlebot_scaling[0], 1.0f);
    nh.param("turtlebot/scaling/angular",   turtlebot_scaling[1], 1.0f);
    nh.param("turtlebot/bounds/linear",     turtlebot_bounds[0],  0.6f);
    nh.param("turtlebot/bounds/angular",    turtlebot_bounds[1],  0.99f);
    nh.param("alphabot/scaling/linear",     alphabot_scaling[0],  0.5f);
    nh.param("alphabot/scaling/angular",    alphabot_scaling[1],  1.0f);
    nh.param("alphabot/bounds/linear",      alphabot_bounds[0],   0.3f);
    nh.param("alphabot/bounds/angular",     alphabot_bounds[1],   0.99f);
    nh.param("gopigo/scaling/linear",       gopigo_scaling[0],    1.0f);
    nh.param("gopigo/scaling/angular",      gopigo_scaling[1],    1.0f);
    nh.param("gopigo/bounds/linear",        gopigo_bounds[0],     0.6f);
    nh.param("gopigo/bounds/angular",       gopigo_bounds[1],     0.99f);
    nh.param("quad/scaling/linear",         quad_scaling[0],      1.0f);
    nh.param("quad/scaling/angular",        quad_scaling[1],      1.0f);
    nh.param("quad/bounds/linear",          quad_bounds[0],       0.6f);
    nh.param("quad/bounds/angular",         quad_bounds[1],       0.99f);

    _devices[DeviceId::JOYSTICK].set_scaling(joystick_scaling[0], joystick_scaling[1]);

    _devices[DeviceId::TURTLEBOT].set_scaling(turtlebot_scaling[0], turtlebot_scaling[1]);
    _devices[DeviceId::TURTLEBOT].set_bounds(turtlebot_bounds[0], turtlebot_bounds[1]);

    _devices[DeviceId::ALPHABOT].set_scaling(alphabot_scaling[0], alphabot_scaling[1]);
    _devices[DeviceId::ALPHABOT].set_bounds(alphabot_bounds[0], alphabot_bounds[1]);

    _devices[DeviceId::GOPIGO].set_scaling(gopigo_scaling[0], gopigo_scaling[1]);
    _devices[DeviceId::GOPIGO].set_bounds(gopigo_bounds[0], gopigo_bounds[1]);

    _devices[DeviceId::QUAD].set_scaling(quad_scaling[0], quad_scaling[1]);
    _devices[DeviceId::QUAD].set_bounds(quad_bounds[0], quad_bounds[1]);
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
    ROS_INFO("ui_set_mode_cb: %d, %d", set_mode.device.id, set_mode.mode.data);
    _devices[set_mode.device.id].set_mode(set_mode.mode.data);
}

void CentralNode::ui_command_req_cb(const geometry_msgs::Twist& cmd) {
    ROS_INFO_STREAM("ui_cmd_req_cb: " << cmd);
    _ui_cmd_req = cmd;
    _ui_cmd_req.linear.x *= ui_scaling[0];
    _ui_cmd_req.angular.z *= ui_scaling[1];
    _ui_cmd_req_updated = true;
}

void CentralNode::ui_publish_device_status() {
    for (int i = 0; i < _devices.size(); ++i) {
        _device_status_list.status_list[i] = _devices[i].status();
    }
    _ui_device_status_pub.publish(_device_status_list);
    ROS_INFO_STREAM("ui_publish_device_status: " << _device_status_list);
}

void CentralNode::publish_mode_req_to_device() const {
    ROS_INFO("publish_mode_req_to_device");
    for (const Device& device : _devices) {
        device.publish_mode_req();
    }
}

void CentralNode::publish_command_to_devices() {
    bool device_source = false;
    for (Device& device : _devices) {
        if (device.mode().data & Mode::SOURCE) {
            device_source = true;
            _cmd_vel = device.get_cmd_req();
            break;
        }
    }

    if (!device_source && !_ui_cmd_req_updated) {
        return;
    }

    _ui_cmd_req_updated = false;
    _stale_command = false;

    _timer.stop();
    _timer.start();

    if (!device_source) {
        _cmd_vel = _ui_cmd_req;
    }

    if (_stale_command) {
        return;
    }

    int i = 0;
    for (const Device& device : _devices) {
        device.publish_command(_cmd_vel);
        ROS_INFO_STREAM("ui_publish_command_to_devices: " << _cmd_vel << " from " << i);
        ++i;
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
