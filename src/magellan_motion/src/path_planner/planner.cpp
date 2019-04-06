#include "planner.h"

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

static const double SQRT2_MINUS_ONE = 0.4142135624;
static const double SQRT2 = 1.4142135624;
static const double INF_VALUE = 999999;

// 8-connected grid
static const int NUMOFDIRS = 8;
static const int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
static const int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
static const double costs[NUMOFDIRS] = {SQRT2,  1,  SQRT2, 1,  1, SQRT2, 1, SQRT2};

using namespace MagellanPlanner;

PathPlanner::PathPlanner(ros::NodeHandle& nh, double resolution)
    :startX(0),
    startY(0),
    goalX(0),
    goalY(0),
    _resolution(resolution),
    open_(comp_),
    graph(10/resolution, std::vector<Successor>(10/resolution, {false, INF_VALUE, INF_VALUE, 0, 0}))
{
    mapSize = 10/resolution;
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("fake_obstacles", 0, &PathPlanner::mapCallback);
    _has_map = false;
}

int PathPlanner::getKey(double x, double y) {
    int xMap = (int) x/_resolution;
    int yMap = (int) y/_resolution;
    return ((xMapy + yMap)*(xMap + yMap + 1)/2 + yMap);
}

Path PathPlanner::plan(Point goal) {
    goalX = goal.x;
    goalY = goal.y;

    Path p;

    if (!isFree(goalX, goalY)) {
        ROS_ERROR("PathPlanner: Goal is not free!!!!");
        throw std::runtime_error("PathPlanner: Goal is not free!");
    }

    std::shared_ptr<Successor> start = std::make_shared<Successor>();
    start->hCost = getHeuristic(0, 0);
    start->gCost = 0;
    start->xPose = 0;
    start->yPose = 0;
    start->key = getKey(0,0);
    start->closed = false;

    open_.push(start);
    nodes.insert({start->key, start});

    bool goalFound = false;

    int numExpand = 0;

    while (ros::ok() && !open_.empy() && !goalFound) {
        numExpand++;
        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            // generate all valid successors
            int newx = next->xPose + dX[dir];
            int newy = next->yPose + dY[dir];
            double cc = costs[dir] + next->gCost;

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){
                // if inside map
                if (isFree(newx,newy,x_size,y_size)){
                    // if free
                    /*PlannerNode * newSuc = &mapNodes[newx][newy];
                    if (newSuc->gCost > cc){
                        // gCost of successor is larger than new g cost
                        newSuc->gCost = cc;
                        newSuc->hCost = heuristic(newx, newy, goalposeX, goalposeY);
                        if (newSuc->closed != 1){
                            q.push(newSuc);
                        }
                    }*/
                }
            }
        }

        std::shared_ptr<Successor> next = open_.top();
        open_.pop();

        if (!next->closed) {
            next->closed = true;


        }



        bool alreadyOpen = (nodes.count(next->key) !=0);




    }


    return p;
}

bool PathPlanner::isFree(double x, double y) {
    int mapWidth = _map.info.width;
    double mapResolution = _map.info.resolution;

    // origin isnt needed because the origin should be center
    // of the robot and x, y should be from robot center
    // the future we should be more general in our frames

    if (_resolution != mapResolution) {
        ROS_WARN_THROTTLE(1, "Path Planner: resoltuion from msg does not match what was expected");
    }

    // convert x, y to points in map
    int xMap = (int) x/mapResolution;
    int yMap = (int) y/mapResolution;

    if (xMap<0 || yMap<0) {
        ROS_ERROR("Path Planner: X and Y cell in isFree is negative");
        throw ros::InvalidParameterException("X and Y cell in isFree are negative");
    }

    if (xMap>mapWidth || yMap>mapWidth) {
        ROS_ERROR_STREAM("PathPlanner: X or Y cells are outside map; assuming free. X = " << xMap << " Y = " << yMap);
        return true;
    }

    // convert to row major order
    int index = yMap * mapWidth + xMap;

    ROS_INFO_STREAM("PathPlanner: index = " << index);

    return (_map.data[index] != 100);
}

double PathPlanner::getHeuristic(double x, double y) {
    // eight connected grid
    double h = (SQRT2_MINUS_ONE * MIN(abs(goalX-x), abs(goalY-y)) +
                MAX(abs(goalX-x), abs(goalY-y)));
    return h;
}

void PathPlanner::resetGraph() {
    Successor defaultNode = {false, INF_VALUE, INF_VALUE, 0, 0};
    std::fill(graph.begin(), graph.end(), defaultNode);
}

bool PathPlanner::isGoal(double x, double y) {
    double dist = std::sqrt(std::pow((x-goalX), 2)) + std::sqrt(std::pow((y-goalY), 2));

    return (dist<=_resolution);
}

void PathPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    _map = *msg;
    _has_map = true;
}
