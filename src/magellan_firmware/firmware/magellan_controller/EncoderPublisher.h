#ifndef ENCODER_PUBLISHER_H
#define ENCODER_PUBLISHER_H

#include <ros.h>
#include <magellan_core/EncoderDeltaStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include "Rate.h"

class EncoderPublisher {
public:
    EncoderPublisher(ros::NodeHandle& nh);
    void Update(bool forward);
private:
    ros::NodeHandle& nh_;
    magellan_core::EncoderDeltaStamped encoder_msg_;
    ros::Publisher encoder_publisher_;
    geometry_msgs::TwistWithCovarianceStamped twist_msg_;
    ros::Publisher velocity_publisher_;
    Rate update_rate_;
    long int last_left_count_;
    long int last_right_count_;
};

#endif
