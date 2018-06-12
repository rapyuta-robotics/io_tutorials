#include "device_wrapper.hpp"

namespace simon_says {

class GoPiGo : public DeviceWrapper {
public:
  explicit GoPiGo(ros::NodeHandle &nh) : DeviceWrapper(nh) {
    _gopigo_cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _gopigo_cmd_req_sub =
        nh.subscribe("reaction", 20, &GoPiGo::command_req_cb, this);
  }

  void command_cb_internal(const geometry_msgs::Twist &cmd) override {
    _gopigo_cmd_pub.publish(cmd);
  }

  void command_req_cb(const geometry_msgs::Twist &cmd_req) {
    publish_command_request(cmd_req);
  }

private:
  ros::Publisher _gopigo_cmd_pub;
  ros::Subscriber _gopigo_cmd_req_sub;
};

} // namespace simon_says

int main(int argc, char **argv) {
  ros::init(argc, argv, "gopigo_node");

  double freq;

  ros::NodeHandle nh, priv_nh("~");
  priv_nh.param("freq", freq, 20.0);

  simon_says::GoPiGo node(nh);

  ros::Rate rate(freq);

  while (ros::ok()) {
    node.publish_status();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
