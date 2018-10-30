#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

class PathFollower {
    public:
        PathFollower(ros::NodeHandle& nh);
    private:
        ros::NodeHandle& nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        ros::Subscriber path_subscriber_;

        nav_msgs::Path::ConstPtr current_path_;
        tf2::Vector3 current_position_;
        int path_start_index_;

        void UpdatePath(nav_msgs::Path::ConstPtr path);
};

#endif
