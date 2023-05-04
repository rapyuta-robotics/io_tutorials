#include <io_turtle_sim_env/io_turtle_sim_env_scoped_targetted.hpp>

namespace turtlesim {

SimTurtleScopedTargetted::SimTurtleScopedTargetted(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, int id, TurtleGeometryMsg turtle, 
        float radius, float collision_range, float x_min, float x_max, float y_min, float y_max)
    :  SimTurtleBase<TurtleGeometryMsg>(nh, priv_nh, id, turtle, radius, collision_range, x_min, x_max, y_min, y_max){

    // ROS topic queue size
    int queue_size;

    // Initialize parameters
    priv_nh.param("queue_size", queue_size, 100);

    // Construct ROS subscribers and publishers
    _velocity_sub = nh.subscribe("turtle"+std::to_string(_id)+"/sim/cmd_vel", queue_size, &SimTurtleScopedTargetted::velocity_callback, this);
    _pose_pub = new ros::Publisher(nh.advertise<geometry_msgs::Pose2D>("turtle"+std::to_string(_id)+"/sim/pose", queue_size));
    _sensors_pub = new ros::Publisher(nh.advertise<io_turtle_msgs::DistanceSensor>("turtle"+std::to_string(_id)+"/sim/sensors", queue_size));

}

SimTurtleScopedTargetted::~SimTurtleScopedTargetted(){
    delete _pose_pub;
    delete _sensors_pub;
}

void SimTurtleScopedTargetted::velocity_callback(const geometry_msgs::Twist& vel) {
    _turtle.set_cmd_vel(vel.linear.x, vel.angular.z);
}

SimEnvScopedTargetted::SimEnvScopedTargetted(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, float time_step)
        : SimEnvBase(nh, priv_nh, time_step) {
    // _register_service = _nh.advertiseService("register_sim_turtle", &SimEnvBase::register_turtle, this);
    // ROS_INFO("'Register simulation turtle' service started.");
}

void SimEnvScopedTargetted::register_turtle_impl(int id){
    TurtleGeometryMsg new_turtle = TurtleGeometryMsg();
    new_turtle._pose.x = random_value(_x_min + _collision_range, _x_max - _collision_range);
    new_turtle._pose.y = random_value(_y_min + _collision_range, _y_max - _collision_range);
    new_turtle._pose.theta = 0.0f;

    // _turtles.emplace(req.id, new_simturtle);
    _turtles.emplace(std::piecewise_construct, 
                        std::forward_as_tuple(id),
                        std::forward_as_tuple(_nh, _priv_nh, id, new_turtle, _turtle_radius, _collision_range, _x_min, _x_max, _y_min, _y_max));

    ROS_INFO_STREAM("New turtle detected: Turtle" << id <<  " and initialized at (" 
        << new_turtle._pose.x << ',' << new_turtle._pose.y << ',' << new_turtle._pose.theta << ").");

}

}  // namespace turtlesim