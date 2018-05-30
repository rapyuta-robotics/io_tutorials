#ifndef SIMON_SAYS_DEVICE_WRAPPER_H
#define SIMON_SAYS_DEVICE_WRAPPER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <simon_says/Mode.h>
#include <simon_says/Status.h>

namespace simon_says {

class DeviceWrapper {
public:
    explicit DeviceWrapper(ros::NodeHandle& nh, double timeout = 1.0)
            : _timer(nh.createTimer(ros::Duration(timeout), &DeviceWrapper::timer_cb, this, true, false)) {
        _status.mode.data = Mode::OFF;
        _status.data = Status::RUNNING;

        _mode_sub = nh.subscribe("mode", 20, &DeviceWrapper::mode_cb, this);
        _command_sub = nh.subscribe("command", 20, &DeviceWrapper::command_cb, this);
        _status_pub = nh.advertise<Status>("status", 1);
        _command_req_pub = nh.advertise<geometry_msgs::Twist>("command_req", 1);
    }

    // Override to handle command messages
    virtual void command_cb_internal(const geometry_msgs::Twist& cmd) {}

    // Override to handle mode messages
    virtual bool mode_cb_internal(const Mode& mode) {
        return true;
    }

    void publish_status() {
        _status_pub.publish(_status);
    }

    void publish_command_request(const geometry_msgs::Twist& cmd_req) {
        if (_status.mode.data & Mode::SOURCE) {
            _command_req_pub.publish(cmd_req);
        }
    }

    void timer_cb(const ros::TimerEvent& event) {
        stop_robot();
    }

    void stop_robot() {
        command_cb_internal(geometry_msgs::Twist());
    }

    void mode_cb(const Mode& mode) {
        if (_status.mode.data != mode.data) {
            if (!(mode.data & Mode::SINK)) {
                stop_robot();
            }
            if (mode_cb_internal(mode)) {
                _status.mode = mode;
            }
        }
    }

    void command_cb(const geometry_msgs::Twist& cmd) {
        if ((_status.data == Status::RUNNING) && (_status.mode.data & Mode::SINK)) {
            _timer.stop();
            _timer.start();
            command_cb_internal(cmd);
        }
    }

    ros::Publisher _status_pub;
    ros::Publisher _command_req_pub;

    ros::Subscriber _mode_sub;
    ros::Subscriber _command_sub;

    ros::Timer _timer;
    Status _status;
};
}

#endif /* SIMON_SAYS_DEVICE_WRAPPER_H */
