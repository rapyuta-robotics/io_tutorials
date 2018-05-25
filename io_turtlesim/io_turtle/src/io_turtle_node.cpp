#include <actionlib/server/simple_action_server.h>

#include <io_turtle/io_turtle.hpp>
#include <io_turtle_services/RegisterTurtle.h>
#include <io_turtle_services/RegisterSimTurtle.h>
#include <io_turtle_action/GoToAction.h>

using namespace io_turtle_action;
using namespace io_turtle_services;

class GoToActionServer {
public:
    GoToActionServer(ros::NodeHandle& nh, const std::string& server_name, turtlesim::IOTurtle* io_turtle,
            float frequency, float timeout)
            : _action_server(nh, server_name, boost::bind(&GoToActionServer::goto_action_callback, this, _1), false)
            , _io_turtle(io_turtle)
            , _frequency(frequency)
            , _preempted(false)
            , _timer(nh.createTimer(ros::Duration(timeout), &GoToActionServer::timer_callback, this, true, false)) {
        _action_server.start();
    }

    void timer_callback(const ros::TimerEvent& event) {
        ROS_INFO("Unable to reach goal.");
        _preempted = true;
    }

    void goto_action_callback(const GoToGoalConstPtr& goal) {
        ros::Rate rate(_frequency);

        _io_turtle->reset();
        _io_turtle->set_goal(goal->x, goal->y);

        while (!_io_turtle->reached_goal()) {
            _io_turtle->move_towards_goal();

            if (!_io_turtle->is_moving() && !_timer.hasPending()) {
                _timer.start();
            } else if (_io_turtle->is_moving() && _timer.hasPending()) {
                _timer.stop();
            }

            // Handle cases where a turtle has been stopped or a goal has been cancelled
            if (_preempted || _action_server.isPreemptRequested() || !ros::ok() || _io_turtle->stop_requested()) {
                ROS_INFO("Goal request cancelled");

                _io_turtle->stop_turtle();
                _action_server.setPreempted();
                _feedback.data = false;
                _preempted = false;
                break;
            }

            _feedback.data = true;
            _action_server.publishFeedback(_feedback);

            rate.sleep();
        }

        // Arrived at goal
        if (_feedback.data) {
            _result.data = true;
            _io_turtle->stop_turtle();
            _action_server.setSucceeded(_result);
        }

        _timer.stop();
    }

private:
    turtlesim::IOTurtle* _io_turtle;

    actionlib::SimpleActionServer<GoToAction> _action_server;
    ros::Timer _timer;

    GoToFeedback _feedback;
    GoToResult _result;

    float _frequency;
    bool _preempted;
};

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "io_turtle_node");
    ROS_INFO("Started Turtle Node.");

    int id;
    bool sim;
    int queue_size;
    float time_step;
    float timeout;

    // Load parameters
    ros::NodeHandle nh, priv_nh("~");
    priv_nh.param("sim", sim, true);
    priv_nh.param("queue_size", queue_size, 100);
    priv_nh.param("time_step", time_step, 0.1f);
    priv_nh.param("goal_timeout", timeout, 5.0f);

    // Set ROS loop rate
    float frequency = 1.0f / time_step;
    ros::Rate rate(frequency);

    // Create turtle
    turtlesim::IOTurtle io_turtle(priv_nh);

    // Set up parameter dynamic reconfiguration
    dynamic_reconfigure::Server<io_turtle::ControlConfig> dr_srv;
    dynamic_reconfigure::Server<io_turtle::ControlConfig>::CallbackType dr_cb;
    dr_cb = boost::bind(&turtlesim::IOTurtle::dynamic_parameters_callback, &io_turtle, _1, _2);
    dr_srv.setCallback(dr_cb);

    // Construct ROS service for registration with command center
    ros::ServiceClient client = nh.serviceClient<RegisterTurtle>("register_turtle");
    client.waitForExistence();

    // Service request
    RegisterTurtle req;

    // Send request
    if (!client.call(req)) {
        ROS_ERROR_STREAM("Unable to make a call to register turtle at the command center.");
        return 1;
    }

    // Successfully register with command center
    id = req.response.id;
    io_turtle.register_turtle(nh, id, sim, queue_size);
    ROS_INFO("Registered myself as Turtle%d", id);

    if (sim) {
        ROS_INFO("Starting node in simulation mode.");

        // Construct ROS service for registration with simulation environment
        ros::ServiceClient register_sim_client = nh.serviceClient<RegisterSimTurtle>("register_sim_turtle");
        register_sim_client.waitForExistence();

        // Service request
        RegisterSimTurtle sim_req;
        sim_req.request.id = id;

        // Send request
        if (!register_sim_client.call(sim_req)) {
            ROS_ERROR("Unable to make a call to register sim turtle service.");
        }

        // Validate response
        if (!sim_req.response.data) {
            ROS_ERROR("Unable to register with simulation environment.");
        }
    }

    // Construct ROS action server for goals
    std::string action_server_name = "turtle_" + std::to_string(id) + "/goto_action";
    GoToActionServer goto_server(nh, action_server_name, &io_turtle, frequency, timeout);

    while (ros::ok()) {
        if (io_turtle.check_collision()) {
            io_turtle.stop_turtle();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
