#include "path_follower.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static inline double L1Norm(const geometry_msgs::PoseStamped& pose) {
    return std::abs(pose.pose.position.x) + std::abs(pose.pose.position.y);
}


PathFollower::PathFollower(ros::NodeHandle& nh) :
    nh_(nh),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    path_subscriber_(nh.subscribe("/path", 10, &PathFollower::UpdatePath, this)),
    path_start_index_(0) {
}

void PathFollower::UpdatePath(nav_msgs::Path::ConstPtr path) {
    auto transform = tf_buffer_.lookupTransform((*path).header.frame_id, "base_link", ros::Time::now(), ros::Duration(1.0));

    geometry_msgs::PoseSamped closest_pose;
    double closest_pose_distance;
    auto it = path.poses.begin();
    for ( it += path_start_index_; it != path.poses.end(); ++it) {
        geometry_msgs::PoseStamped temp;
        tf2::doTransform(*it, temp, transform);
        double pose_distance = L1Norm(pose);
        if ( pose_distance < closest_pose_distance ) {
            closest_pose_distance = pose_distance;
            path_start_index_ = it - path.poses.begin();
        }
    }
    current_path_ = path;
}


void PathFollower::Update() {
    auto transform = tf_buffer_.lookupTransform((*path).header.frame_id, "base_link", ros::Time::now(), ros::Duration(1.0));

    geometry_msgs::PoseSamped closest_pose;
    double last_pose_distance = 999;
    auto it = path.poses.begin();
    for ( it += path_start_index_; it != path.poses.end(); ++it) {
        geometry_msgs::PoseStamped temp;
        tf2::doTransform(*it, temp, transform);
        double pose_distance = L1Norm(*it);
        if ( pose_distance > last_pose_distance ) {
            break;
        }
    }
}
