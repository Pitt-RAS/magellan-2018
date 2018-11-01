#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

class PathFollower {
    public:
        PathFollower(ros::NodeHandle& nh, double discretization, double lookahead_distance);
        void Update();
    private:
        ros::NodeHandle& nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        ros::Rate debug_marker_rate_;

        ros::Subscriber path_subscriber_;
        ros::Publisher velocity_publisher_;
        ros::Publisher turning_radius_publisher_;

        nav_msgs::Path::ConstPtr current_path_;
        tf2::Vector3 current_position_;
        int path_start_index_;

        double discretization_;
        double lookahead_distance_;

        void UpdatePath(nav_msgs::Path::ConstPtr path);
        void UpdateMarkers();
};

#endif
