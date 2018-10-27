#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>

class PathFollower {
    public:
        PathFollower(ros::NodeHandle& nh);
    private:
        ros::NodeHandle& nh_;
        ros::Subscriber path_subscriber_;
        nav_msgs::Path::ConstPtr current_path_;
        int path_start_index_;

        void UpdatePath(nav_msgs::Path::ConstPtr path);
};

#endif
