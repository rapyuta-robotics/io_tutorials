#include <std_msgs/String.h>

#include <io_turtle/io_turtle.hpp>
#include <io_turtle/io_turtle_scoped_targetted.hpp>

int main(int argc, char** argv) {

    bool scoped_targetted = false;
    if(argc>1 and std::string(argv[1]) == "--scoped_targetted"){
        scoped_targetted = true;
    }
    
    // Initialize ROS
    ros::init(argc, argv, "io_turtle_node");
    ros::NodeHandle nh, priv_nh("~");

    if(scoped_targetted){
        ROS_INFO("Started Turtle with Scoped and Targetted Node.");
        turtlesim::IOTurtleScopedTargettedNode node(nh, priv_nh);
        node.spin();
    }
    else{
        ROS_INFO("Started Turtle Node.");
        turtlesim::IOTurtleNode node(nh, priv_nh);
        node.spin();
    }

    return 0;
}
