#include "device.hpp"
#include <cmath>

namespace simon_says {

static void clamp_magnitude(double &x, const double &max) {
  if (std::fabs(x) > max) {
    x = std::copysign(max, x);
  }
}

Device::Device(ros::NodeHandle &nh, const std::string &ns) {
  nh.param("linear_scale", _linear_scale, 1.0f);
  nh.param("angular_scale", _angular_scale, 1.0f);
  nh.param("max_linear", _max_linear, 0.1f);
  nh.param("max_angular", _max_angular, 1.0f);

  float device_timeout;
  int queue_size;

  nh.param("device_timeout", device_timeout, 30.0f);
  nh.param("queue_size", queue_size, 10);

  _timer = nh.createTimer(ros::Duration(device_timeout), &Device::timer_cb,
                          this, true, false);
  _status.data = Status::OFF;
  _status.mode.data = Mode::OFF;
  _status_sub =
      nh.subscribe(ns + "/status", queue_size, &Device::status_cb, this);
  _command_req_sub = nh.subscribe(ns + "/command_req", queue_size,
                                  &Device::command_req_cb, this);
  _mode_pub = nh.advertise<Mode>(ns + "/mode", queue_size);
  _command_pub =
      nh.advertise<geometry_msgs::Twist>(ns + "/command", queue_size);
}

void Device::tune(const std::vector<float> &tunings) {
  float linear_scale = tunings[0];
  float angular_scale = tunings[1];
  float linear_bound = tunings[2];
  float angular_bound = tunings[3];

  // avoid division by zero
  _linear_scale = (std::fabs(linear_scale) < 1e-3) ? 1e-3 : linear_scale;
  _angular_scale = (std::fabs(angular_scale) < 1e-3) ? 1e-3 : angular_scale;

  _max_linear = linear_bound;
  _max_angular = angular_bound;
}

void Device::set_mode(uint8_t data) { _mode_req.data = data; }

const Status &Device::status() const { return _status; }

const Mode &Device::mode() const { return _status.mode; }

const geometry_msgs::Twist &Device::get_cmd_req() const { return _cmd_req; }

void Device::publish_mode_req() const { _mode_pub.publish(_mode_req); }

void Device::publish_command(geometry_msgs::Twist cmd) const {
  if ((_status.data == Status::RUNNING) && (_status.mode.data & Mode::SINK)) {
    cmd.linear.x *= _linear_scale;
    cmd.angular.z *= _angular_scale;
    clamp_magnitude(cmd.linear.x, _max_linear);
    clamp_magnitude(cmd.angular.z, _max_angular);
    _command_pub.publish(cmd);
  }
}

void Device::timer_cb(const ros::TimerEvent &event) {
  _status.data = Status::OFF;
  _status.mode.data = Mode::OFF;
}

void Device::status_cb(const Status &status) {
  _timer.stop();
  _timer.start();
  _status = status;
}

void Device::command_req_cb(const geometry_msgs::Twist &cmd) {
  _cmd_req = cmd;
  _cmd_req.linear.x /= _linear_scale;
  _cmd_req.angular.z /= _angular_scale;
}

} // namespace simon_says
