#include "path_follower.h"
#include <std_msgs/Float64.h>
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
    velocity_publisher_(nh.advertise<std_msgs::Float64>("/platform/cmd_velocity", 10)),
    turning_radius_publisher_(nh.advertise<std_msgs::Float64>("/platform/cmd_turning_radius", 10)),
    markers_(nh, 10),
    current_path_(),
    path_start_index_(0),
    discretization_(discretization),
    lookahead_distance_(lookahead_distance) {
}

void PathFollower::UpdatePath(nav_msgs::Path::ConstPtr path) {
    ROS_WARN("updated path");
    current_path_ = path;
    path_start_index_ = 0;
}

void PathFollower::Update() {
    if ( !current_path_ )
        return;

    if ( (*current_path_).poses.size() == 0 ) {
        ROS_WARN("Commanded path is empty");
        return;
    }

    std::string frame = (*current_path_).header.frame_id;
    ROS_INFO("Transforming from %s", frame.c_str());
    auto transform = tf_buffer_.lookupTransform("base_link", frame, ros::Time::now(), ros::Duration(1.0));

    int lookahead_points = lookahead_distance_ / discretization_;

    geometry_msgs::PoseStamped closest_pose;
    auto it = (*current_path_).poses.begin();
    geometry_msgs::PoseStamped closest_point;
    tf2::doTransform(*it, closest_point, transform);
    geometry_msgs::PoseStamped temp;
    for ( it += 0; it != (*current_path_).poses.end(); ++it) {
        tf2::doTransform(*it, temp, transform);
        if ( L1Norm(temp) > L1Norm(closest_point) )
            break;
        closest_point = temp;
    }
    path_start_index_ = std::distance((*current_path_).poses.begin(), it);

    ROS_WARN("distance is %d %d", std::distance(it, (*current_path_).poses.end()), lookahead_points);
    int jump = std::min(static_cast<int>(std::distance(it, (*current_path_).poses.end())), lookahead_points);
    it += std::min(static_cast<int>(std::distance(it, (*current_path_).poses.end())), lookahead_points);
    ROS_WARN("jumping %d", jump);

    geometry_msgs::PoseStamped lookahead_pose;
    tf2::doTransform(*it, lookahead_pose, transform);

    double turning_radius = 0;
    if ( lookahead_pose.pose.position.x > 0 )
        turning_radius = (lookahead_distance_*lookahead_distance_) / (2.0 * lookahead_pose.pose.position.y);

    ROS_INFO("Error to lookahead is %2.2f", lookahead_pose.pose.position.y);
    static std_msgs::Float64 turning_command;
    turning_command.data = turning_radius;

    turning_radius_publisher_.publish(turning_command);

    markers_.UpdateClosestPoint(closest_point);
    markers_.UpdateLookahead(lookahead_pose);

    markers_.Update();
}
