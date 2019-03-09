#include <ros/ros.h>
#include "path_follower.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double rate_hz = 100;
    private_nh.getParam("rate", rate_hz);
    ros::Rate rate(rate_hz);

    double discretization;
    if ( !private_nh.getParam("discretization", discretization) ) {
        ROS_ERROR("discretization param unset");
        ros::shutdown();
        return 0;
    }

    double max_velocity;
    if ( !private_nh.getParam("max_vel", max_velocity) ) {
        ROS_ERROR("Maximum velocity param unset");
        ros::shutdown();
        return 0;
    }

    double max_acceleration;
    if ( !private_nh.getParam("max_acc", max_acceleration) ) {
        ROS_ERROR("Maximum acceleration param unset");
        ros::shutdown();
        return 0;
    }

    double stanley_gain;
    if ( !private_nh.getParam("stanley_gain", stanley_gain) ) {
        ROS_ERROR("stanley_gain param unset");
        ros::shutdown();
        return 0;
    }

    /*
    double kD;
    if ( !private_nh.getParam("kD", kD) ) {
        ROS_ERROR("kD constant param unset");
        ros::shutdown();
        return 0;
    }
    */

    PathFollower path_follower(nh, discretization, max_velocity, max_acceleration, stanley_gain);

    while (ros::ok()) {
        try {
            path_follower.Update();
        }
        catch (tf2::TransformException e) {
            ROS_ERROR("Failed to lookup transform");
        }
        ros::spinOnce();
        rate.sleep();
    }
}
