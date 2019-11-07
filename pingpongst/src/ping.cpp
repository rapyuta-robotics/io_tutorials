
#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <stdlib.h>
#include <string>

#include <pingpongst/ping.h>

using pingpongst::ping;

ros::Publisher ping_pub;
ros::Subscriber ping_sub;

int counter = 0;
int send_cnt = 0;
int rcv_cnt = 0;
int skipped_cnt = 0;
int miss_cnt = 0;
int wrong_id_cnt = 0;
int duplicates_cnt = 0;

bool should_send = false;
ping out_msg;

float ttl;

ros::Timer ttl_timer;
ros::Timer show_timer;

void send();
void send_timed(const ros::TimerEvent&) {
    ++miss_cnt;
    std::cout << "Resend " << counter << std::endl;
    send();
}

void send() {
    ttl_timer.stop();
    out_msg.val = std::max(out_msg.val, counter);
    should_send = true;
    if (!ros::isShuttingDown()) {
        ros::NodeHandle nh;
        ttl_timer = nh.createTimer(ros::Duration(ttl), send_timed, true, true);
    }
}
void do_publish() {
    if (should_send) {
        if (!ros::isShuttingDown()) {
            ++send_cnt;
            ping_pub.publish(out_msg);
        }
        should_send = false;
    }
}

void show();
void show_timed(const ros::TimerEvent&) {
    show();
}

void show() {
    ROS_INFO_STREAM("Skipped " << skipped_cnt);
    ROS_INFO_STREAM("Missed " << miss_cnt);
    ROS_INFO_STREAM("Wrong source " << wrong_id_cnt);
    ROS_INFO_STREAM("Duplicate " << duplicates_cnt);
    ROS_INFO_STREAM("Total sent " << send_cnt);
    ROS_INFO_STREAM("Total received " << rcv_cnt);
}

void ping_cb(const ping& msg) {
    if (msg.id != out_msg.id) {
        ++wrong_id_cnt;
    } else if (msg.val > counter) {
        skipped_cnt += msg.val - counter - 1;
        counter = msg.val + 1;
        ++rcv_cnt;
        send();
    } else if (msg.val < counter) {
        ++duplicates_cnt;
    }
}

void sighandl(int signum) {
    show_timer.stop();
    ttl_timer.stop();
    ping_sub.shutdown();
    ping_pub.shutdown();
    std::cout << "Terminate with signal " << signum << std::endl;
    show();
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ping", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    signal(SIGINT, sighandl);  // after nodehandle...
    std::string talk_topic = ros::param::param<std::string>("~talk", "ping");
    ping_pub = nh.advertise<ping>(talk_topic, 10);

    bool init = ros::param::param<bool>("~start", true);
    std::string listen_topic = ros::param::param<std::string>("~listen", "ping");
    out_msg.id = ros::param::param<int>("~id", 1);
    ttl = ros::param::param<float>("~ttl", 3.0f);

    ping_sub = nh.subscribe(listen_topic, 10, ping_cb);

    if (init) {
        ros::Duration(0.5).sleep();  // we know for a fact that publishing immediately will fail because the publisher
                                     // is not yet ready.
        // So we wait *sigh*, but we also make sure the message is repeated
        send();
        do_publish();
    } else {
        counter = -1;
    }
    show_timer = nh.createTimer(ros::Duration(5.0), show_timed, false, true);
    ros::Rate r(10);  // 10 hz
    while (ros::ok() && !ros::isShuttingDown()) {
        ros::spinOnce();
        do_publish();
        r.sleep();
    }
    return 0;
}
