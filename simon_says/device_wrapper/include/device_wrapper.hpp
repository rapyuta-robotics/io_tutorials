#ifndef SIMON_SAYS_DEVICE_WRAPPER_H
#define SIMON_SAYS_DEVICE_WRAPPER_H

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <simon_says_msgs/Mode.h>
#include <simon_says_msgs/Status.h>

namespace simon_says {

using geometry_msgs::Twist;
using simon_says_msgs::Mode;
using simon_says_msgs::Status;

class DeviceWrapper {
public:
  explicit DeviceWrapper(ros::NodeHandle &nh, double timeout = 1.0);

  // Override to handle command messages
  virtual void command_cb_internal(const Twist &cmd);

  // Override to handle mode messages
  virtual bool mode_cb_internal(const Mode &mode);

  void stop_robot();
  void publish_status();
  void publish_command_request(const Twist &cmd_req);

  void timer_cb(const ros::TimerEvent &event);
  void mode_cb(const Mode &mode);
  void command_cb(const Twist &cmd);

  ros::Publisher _status_pub;
  ros::Publisher _command_req_pub;

  ros::Subscriber _mode_sub;
  ros::Subscriber _command_sub;

  ros::Timer _timer;
  Status _status;
};
} // namespace simon_says

#endif /* SIMON_SAYS_DEVICE_WRAPPER_H */
