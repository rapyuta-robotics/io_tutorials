#include "simon_says/device_wrapper.hpp"

//< @todo Move this file to Kobuki package

namespace simon_says {

class TurtleBot : public DeviceWrapper {
public:
    explicit TurtleBot(ros::NodeHandle& nh, double timeout) : DeviceWrapper(nh, timeout) {
        _turtlebot_cmd_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    }

    void command_cb_internal(const geometry_msgs::Twist& cmd) {
        _turtlebot_cmd_pub.publish(cmd);
    }

private:
    ros::Publisher _turtlebot_cmd_pub;
};

}  // namespace simon_says

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "turtle_bot_node");
    double freq, timeout;

    // Load parameters
    ros::NodeHandle nh, priv_nh("~");
    priv_nh.param("freq", freq, 20.0);
    priv_nh.param("timeout", timeout, 1.0);

    // Construct TurtleBot node
    simon_says::TurtleBot node(nh, timeout);

    // Set ROS loop rate
    ros::Rate rate(freq);

    while (ros::ok()) {
        node.publish_status();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
