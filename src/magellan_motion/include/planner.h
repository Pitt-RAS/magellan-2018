#ifndef MAGELLAN_PLANNER_H
#define MAGELLAN_PLANNER_H

#include <ros/ros.h>
#include <time.h>
#include <functional>
#include <queue>

// message header
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

using geometry_msgs::Point;
using nav_msgs::Path;

namespace MagellanPlanner {
// struct to define nodes for planners
struct Successor {
    bool free;     // free or obstacle
    bool closed;
    double gCost;     // g* value
    double hCost;     // H value
    double xPose;
    double yPose;
};

class PathPlanner {
public:
    PathPlanner();
    Path plan(Point start, Point goal);
private:
    double getHeuristic(double x, double y);
    void resetMap();
    void resetGraph();

    std::vector< std::vector<bool> > map;
    std::vector< std::vector<MagellanPlanner::Successor> > graph;
    double startX;
    double startY;
    double goalX;
    double goalY;
};
}
#endif // MAGELLAN_PLANNER_H
