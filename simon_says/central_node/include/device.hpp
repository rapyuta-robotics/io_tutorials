#ifndef SIMON_SAYS_DEVICE_HPP
#define SIMON_SAYS_DEVICE_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <simon_says_msgs/Mode.h>
#include <simon_says_msgs/Status.h>

using namespace simon_says_msgs;

namespace simon_says {

class Device {
public:
  Device();
  Device(ros::NodeHandle &nh, const std::string &ns);

  void tune(const std::vector<float> &tunings);
  void set_mode(uint8_t data);

  const Status &status() const;
  const Mode &mode() const;

  const geometry_msgs::Twist &get_cmd_req() const;

  void publish_mode_req() const;
  void publish_command(geometry_msgs::Twist cmd) const;

private:
  void timer_cb(const ros::TimerEvent &event);
  void status_cb(const Status &status);
  void command_req_cb(const geometry_msgs::Twist &cmd);

  ros::Subscriber _status_sub;
  ros::Subscriber _command_req_sub;
  ros::Publisher _mode_pub;
  ros::Publisher _command_pub;

  Status _status;
  Mode _mode_req;

  geometry_msgs::Twist _cmd_req;

  float _linear_scale;
  float _angular_scale;
  float _max_linear;
  float _max_angular;

  ros::Timer _timer;
};

} // namespace simon_says

#endif /* SIMON_SAYS_DEVICE_HPP */
