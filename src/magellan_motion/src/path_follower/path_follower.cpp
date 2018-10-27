#include "path_follower.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PathFollower::PathFollower(ros::NodeHandle& nh) :
    nh_(nh),
    path_subscriber_(nh.subscribe("/path", 10, &PathFollower::UpdatePath, this)),
    path_start_index_(0) {
}

void PathFollower::UpdatePath(nav_msgs::Path::ConstPtr path) {
    // Trim the path to start at the current point
    current_path_ = path;
    tf2::Vector3 point;
}
