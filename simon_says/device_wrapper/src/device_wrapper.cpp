#include "device_wrapper.hpp"

namespace simon_says {

DeviceWrapper::DeviceWrapper(ros::NodeHandle &nh, double timeout)
    : _timer(nh.createTimer(ros::Duration(timeout), &DeviceWrapper::timer_cb,
                            this, true, false)) {
  _status.mode.data = Mode::OFF;
  _status.data = Status::RUNNING;

  _mode_sub = nh.subscribe("mode", 20, &DeviceWrapper::mode_cb, this);
  _command_sub = nh.subscribe("command", 20, &DeviceWrapper::command_cb, this);
  _status_pub = nh.advertise<Status>("status", 1);
  _command_req_pub = nh.advertise<Twist>("command_req", 1);
}

// Override to handle command messages
void DeviceWrapper::command_cb_internal(const Twist &cmd) {}

// Override to handle mode messages
bool DeviceWrapper::mode_cb_internal(const Mode &mode) { return true; }

void DeviceWrapper::publish_status() { _status_pub.publish(_status); }

void DeviceWrapper::publish_command_request(const Twist &cmd_req) {
  if (_status.mode.data & Mode::SOURCE) {
    _command_req_pub.publish(cmd_req);
  }
}

void DeviceWrapper::timer_cb(const ros::TimerEvent &event) { stop_robot(); }

void DeviceWrapper::stop_robot() { command_cb_internal(Twist()); }

void DeviceWrapper::mode_cb(const Mode &mode) {
  if (!(mode.data & Mode::SINK)) {
    stop_robot();
  }

  if (mode_cb_internal(mode)) {
    _status.mode = mode;
  }
}

void DeviceWrapper::command_cb(const Twist &cmd) {
  if ((_status.data == Status::RUNNING) && (_status.mode.data & Mode::SINK)) {
    _timer.stop();
    _timer.start();
    command_cb_internal(cmd);
  }
}

} // namespace simon_says
