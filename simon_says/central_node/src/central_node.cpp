#include "central_node.hpp"

namespace simon_says {

CentralNode::CentralNode(ros::NodeHandle &nh)
    : _ui_cmd_req_updated(false), _stale_command(false) {
  int queue_size;
  nh.param("queue_size", queue_size, 10);

  float central_timeout;
  nh.param("central_timeout", central_timeout, 1.0f);

  _timer = nh.createTimer(ros::Duration(central_timeout),
                          &CentralNode::timer_cb, this, true, false);
  _ui_set_mode_sub = nh.subscribe("/ui/set_mode", queue_size,
                                  &CentralNode::ui_set_mode_cb, this);
  _ui_command_req_sub = nh.subscribe("/ui/command", queue_size,
                                     &CentralNode::ui_command_req_cb, this);
  _ui_device_status_pub =
      nh.advertise<StatusList>("/ui/device_status", queue_size);

  nh.param("ui", _ui_scaling);

  std::vector<std::string> device_strings;
  nh.param("device_strings", device_strings);

  _devices.reserve(device_strings.size());
  for (const std::string &name : device_strings) {
    std::vector<float> device_tunings;
    nh.param("devices/" + name, device_tunings);
    _devices.emplace_back(nh, name);
    _devices.back().tune(device_tunings);
  }

  _device_status_list.status_list.reserve(device_strings.size());
  for (int i = 0; i < device_strings.size(); ++i) {
    _device_status_list.status_list.emplace_back(Status());
  }
}

void CentralNode::update() {
  ui_publish_device_status();
  publish_mode_req_to_device();
  publish_command_to_devices();
}

void CentralNode::timer_cb(const ros::TimerEvent &event) {
  _stale_command = true;
}

void CentralNode::ui_set_mode_cb(const SetMode &set_mode) {
  _devices[set_mode.device.id].set_mode(set_mode.mode.data);
}

void CentralNode::ui_command_req_cb(const geometry_msgs::Twist &cmd) {
  _stale_command = false;

  _ui_cmd_req = cmd;
  _ui_cmd_req.linear.x *= _ui_scaling[0];
  _ui_cmd_req.angular.z *= _ui_scaling[1];
  _ui_cmd_req_updated = true;
}

void CentralNode::ui_publish_device_status() {
  for (int i = 0; i < _devices.size(); ++i) {
    _device_status_list.status_list[i] = _devices[i].status();
  }
  _ui_device_status_pub.publish(_device_status_list);
}

void CentralNode::publish_mode_req_to_device() const {
  for (const Device &device : _devices) {
    device.publish_mode_req();
  }
}

void CentralNode::publish_command_to_devices() {
  bool device_source = false;
  for (const Device &device : _devices) {
    if (device.mode().data & Mode::SOURCE) {
      _stale_command = false;
      device_source = true;
      _cmd_vel = device.get_cmd_req();
      break;
    }
  }

  if (!device_source && !_ui_cmd_req_updated) {
    return;
  }
  _ui_cmd_req_updated = false;

  if (_stale_command) {
    return;
  }

  _timer.stop();
  _timer.start();

  if (!device_source) {
    _cmd_vel = _ui_cmd_req;
  }

  for (const Device &device : _devices) {
    device.publish_command(_cmd_vel);
  }
}

} // namespace simon_says

int main(int argc, char **argv) {
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
