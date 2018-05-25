#include <ros/ros.h>
#include <io_turtle_services/RegisterTurtle.h>

namespace turtlesim {

using io_turtle_services::RegisterTurtle;

class TurtleCommandCenter {
public:
    TurtleCommandCenter(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
            : _num_of_turtles(0)
            , _max_turtles(0) {
        priv_nh.param("max_turtles", _max_turtles, 8);
        _register_service = nh.advertiseService("register_turtle", &TurtleCommandCenter::register_new_turtle, this);
        ROS_INFO("Register turtle service started.");
    }

private:
    bool register_new_turtle(RegisterTurtle::Request& req, RegisterTurtle::Response& res) {
        if (_num_of_turtles > _max_turtles - 1) {
            ROS_ERROR(
                    "Cannot register new turtle. Maximum number of turtles that can be registered is %d", _max_turtles);
            return false;
        }

        res.id = _num_of_turtles++;

        ROS_INFO("Registered new turtle: Turtle%d", res.id);
        return true;
    }

    ros::ServiceServer _register_service;
    int _num_of_turtles;
    int _max_turtles;
};

}  // namespace turtlesim

int main(int argc, char** argv) {
    ros::init(argc, argv, "io_turtle_command_center_node");
    ROS_INFO("Started Command Center Node.");

    ros::NodeHandle nh, priv_nh("~");
    turtlesim::TurtleCommandCenter cc(nh, priv_nh);

    ros::spin();

    return 0;
}
