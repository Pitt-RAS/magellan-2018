#include "MagellanSim.h"

MagellanSim::MagellanSim(ros::NodeHandle& nh) :
        node_handle_(nh),
        commanded_velocity_(0.0),
        commanded_radius_(0.0),
        velocity_(0.0),
        steering_angle_(0.0),
        refresh_rate_(10.0) {
    throttle_subscriber_ =
        node_handle_.subscribe("/platform/cmd_velocity", 1, &MagellanSim::UpdateThrottle, this);
    steering_subscriber_ =
        node_handle_.subscribe("platform/cmd_turning_radius", 1, &MagellanSim::UpdateSteering, this);
    velocity_publisher_ =
        node_handle_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/platform/velocity", 1, true);
    turning_radius_publisher_ =
        node_handle_.advertise<std_msgs::Float64>("/platform/turning_radius", 1, true);
    time_ = ros::Time::now();
}

void MagellanSim::run() {
    while (node_handle_.ok()) {
        ros::spinOnce();
        Update();
        ROS_INFO("Velocity: %0.2f\t Turning Radius: %0.2f", velocity_, commanded_radius_);
        refresh_rate_.sleep();
        time_ = ros::Time::now();
    }
}

/*
 * Member function to have the sim update and publish simulated values
 */
void MagellanSim::Update() {
    UpdateVelocity();
    UpdateYaw();
}

void MagellanSim::UpdateVelocity() {
    geometry_msgs::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = time_;
    twist_msg.twist.twist.linear.x = velocity_;
    velocity_publisher_.publish(twist_msg);
}

void MagellanSim::UpdateYaw() {
    std_msgs::Float64 course_msg;
    course_msg.data = commanded_radius_;
    turning_radius_publisher_.publish(course_msg);
}

/*
 * Updates commanded velocity from message and then updates sim's velocity
 * At this point, this member function just matches the velocity as doing the math
 * Seem to have not a significant enough difference to justify complexity
 */
void MagellanSim::UpdateThrottle(const std_msgs::Float64& cmd_velocity) {
    commanded_velocity_ = cmd_velocity.data;
    velocity_ = commanded_velocity_;
}

/*
 * Updates commanded velocity from message and upodates sim's turning radius
 * At this point the sim just immediately matches the commanded turning cmd_radius
 */
void MagellanSim::UpdateSteering(const std_msgs::Float64& cmd_radius) {
    commanded_radius_ = cmd_radius.data;
}
