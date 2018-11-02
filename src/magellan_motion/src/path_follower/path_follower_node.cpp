#include <ros/ros.h>
#include "path_follower.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double rate_hz = 100;
    private_nh.getParam("rate", rate_hz);
    ros::Rate rate(rate_hz);

    double lookahead_distance;
    if ( !private_nh.getParam("lookahead_distance", lookahead_distance) ) {
        ROS_ERROR("lookahead_distance param unset");
        ros::shutdown();
        return 0;
    }

    double discretization;
    if ( !private_nh.getParam("discretization", discretization) ) {
        ROS_ERROR("discretization param unset");
        ros::shutdown();
        return 0;
    }

    PathFollower path_follower(nh, discretization, lookahead_distance);

    while (ros::ok()) {
        path_follower.Update();
        ros::spinOnce();
        rate.sleep();
    }
}
