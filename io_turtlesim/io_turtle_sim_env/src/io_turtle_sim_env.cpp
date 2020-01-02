#include <io_turtle_sim_env/io_turtle_sim_env.hpp>

namespace turtlesim {

SimTurtle::SimTurtle(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, int id, TurtleCustomMsg turtle, 
        float radius, float collision_range, float x_min, float x_max, float y_min, float y_max,
        ros::Publisher* pose_pub,  ros::Publisher* sensors_pub)
    :  SimTurtleBase<TurtleCustomMsg>(nh, priv_nh, id, turtle, radius, collision_range, x_min, x_max, y_min, y_max){
    _pose_pub = pose_pub;
    _sensors_pub = sensors_pub;
}
SimTurtle::~SimTurtle(){
}

SimEnv::SimEnv(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, float time_step)
        : SimEnvBase(nh, priv_nh, time_step) {

    // ROS topic queue size
    int queue_size;

    // Initialize parameters
    priv_nh.param("queue_size", queue_size, 100);

    // Construct ROS subscribers and publishers
    _velocity_sub = nh.subscribe("sim/cmd_vel", queue_size, &SimEnv::velocity_callback, this);
    _pose_pub = nh.advertise<io_turtle_msgs::Pose>("sim/pose", queue_size);
    _sensors_pub = nh.advertise<io_turtle_msgs::DistanceSensor>("sim/sensors", queue_size);
}

void SimEnv::register_turtle_impl(int id){
    TurtleCustomMsg new_turtle = TurtleCustomMsg(id);
    new_turtle._pose.x = random_value(_x_min + _collision_range, _x_max - _collision_range);
    new_turtle._pose.y = random_value(_y_min + _collision_range, _y_max - _collision_range);
    new_turtle._pose.theta = 0.0f;

    // _turtles.emplace(req.id, new_simturtle);
    _turtles.emplace(std::piecewise_construct, 
                        std::forward_as_tuple(id),
                        std::forward_as_tuple(_nh, _priv_nh, id, new_turtle, _turtle_radius, _collision_range, _x_min, _x_max, _y_min, _y_max, &_pose_pub, &_sensors_pub));

    ROS_INFO_STREAM("New turtle detected: Turtle" << id <<  " and initialized at (" 
        << new_turtle._pose.x << ',' << new_turtle._pose.y << ',' << new_turtle._pose.theta << ").");

}

void SimEnv::velocity_callback(const io_turtle_msgs::Velocity& vel) {
    TurtleMap::iterator turtle_it = _turtles.find(vel.id);
    if (turtle_it == _turtles.end()) {
        ROS_ERROR("Turtle%d not registered in simulation.", vel.id);
    } else {
        turtle_it->second.set_cmd_vel(vel.linear_velocity, vel.angular_velocity);
    }
}

}  // namespace turtlesim#include <cmath>