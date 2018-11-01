#include "path_follower.h"
#include <std_msgs/Float64.h>
//#include <vizualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <optional>

static inline double L1Norm(const geometry_msgs::PoseStamped& pose) {
    return std::abs(pose.pose.position.x) + std::abs(pose.pose.position.y);
}


PathFollower::PathFollower(ros::NodeHandle& nh, double discretization, double lookahead_distance) :
    nh_(nh),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    path_subscriber_(nh.subscribe("/path", 10, &PathFollower::UpdatePath, this)),
    velocity_publisher_(nh.advertise<std_msgs::Float64>("/cmd_velocity", 10)),
    turning_radius_publisher_(nh.advertise<std_msgs::Float64>("/cmd_turning_radius", 10)),
    path_start_index_(0),
    discretization_(discretization),
    lookahead_distance_(lookahead_distance) {
}

void PathFollower::UpdatePath(nav_msgs::Path::ConstPtr path) {
    current_path_ = path;
    path_start_index_ = 0;
}

void PathFollower::Update() {
    auto transform = tf_buffer_.lookupTransform((*current_path_).header.frame_id, "base_link", ros::Time::now(), ros::Duration(1.0));

    int lookahead_points = lookahead_distance_ / discretization_;

    geometry_msgs::PoseStamped closest_pose;
    auto it = (*current_path_).poses.begin();
    std::optional<geometry_msgs::PoseStamped> closest_point;
    for ( it += path_start_index_; it != (*current_path_).poses.end(); ++it) {
        geometry_msgs::PoseStamped temp;
        tf2::doTransform(*it, temp, transform);
        if ( closest_point.has_value() && L1Norm(temp) > L1Norm(*closest_point) )
            break;
        closest_point = temp;
    }
    path_start_index_ = std::distance((*current_path_).poses.begin(), it);

    it += std::min(static_cast<int>(std::distance(it, (*current_path_).poses.end())), lookahead_points);

    geometry_msgs::PoseStamped lookahead_pose;
    tf2::doTransform(*it, lookahead_pose, transform);
    double turning_radius = (lookahead_distance_*lookahead_distance_) / (2.0 * lookahead_pose.pose.position.x);

    static std_msgs::Float64 turning_command;
    turning_command.data = turning_radius;

    turning_radius_publisher_.publish(turning_command);
}


void PathFollower::UpdateMarkers() {
//    static vizualization_msgs::Marker closest_point_marker;
//    static vizualization_msgs::Marker lookahead_marker;
}
