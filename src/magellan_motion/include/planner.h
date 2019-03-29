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
    bool isGoal(double x, double y);

    std::vector< std::vector<bool> > map;
    std::vector< std::vector<MagellanPlanner::Successor> > graph;

    std::function<bool(const std::shared_ptr<const Successor>&,
                       const std::shared_ptr<const Successor>&)>
        comp_ = [](const std::shared_ptr<const Successor>& a,
                   const std::shared_ptr<const Succesor>& b) {
            return (a_-> gCost + a->hCost) > (b->gCost + b->hCost);
        };
    
    std::priority_queue<std::shared_ptr<Successor>,
                        std::vector<std::shared_ptr<Successor>>,
                        decltype(comp_)>
        open_;

    double startX;
    double startY;
    double goalX;
    double goalY;
    double _resolution;
};
}
#endif // MAGELLAN_PLANNER_H
