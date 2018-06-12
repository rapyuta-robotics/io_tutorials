#include "device_wrapper.hpp"

namespace simon_says {

class TurtleBot : public DeviceWrapper {
public:
  explicit TurtleBot(ros::NodeHandle &nh, double timeout)
      : DeviceWrapper(nh, timeout) {
    _turtlebot_cmd_pub =
        nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void command_cb_internal(const geometry_msgs::Twist &cmd) override {
    _turtlebot_cmd_pub.publish(cmd);
  }

private:
  ros::Publisher _turtlebot_cmd_pub;
};

} // namespace simon_says

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle_bot_node");

  double freq, timeout;
  ros::NodeHandle nh, priv_nh("~");
  priv_nh.param("freq", freq, 20.0);
  priv_nh.param("timeout", timeout, 1.0);

  simon_says::TurtleBot node(nh, timeout);

  ros::Rate rate(freq);

  while (ros::ok()) {
    node.publish_status();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
