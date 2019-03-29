#include "planner.h"

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

static const double SQRT2_MINUS_ONE = 0.4142135624;
static const double SQRT2 = 1.4142135624;

// 8-connected grid
static const int NUMOFDIRS = 8;
static const int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
static const int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
static const double costs[NUMOFDIRS] = {SQRT2,  1,  SQRT2, 1,  1, SQRT2, 1, SQRT2};

using namespace MagellanPlanner;

PathPlanner::PathPlanner()
    :startX(0),
    startY(0),
    goalX(0),
    goalY(0),


{
    resetMap();
    resetGraph();
}

Path PathPlanner::plan(Point start, Point goal) {
    goalX = goal.x;
    goalY = goal.y;


    Path p;
    return p;
}

double PathPlanner::getHeuristic(double x, double y) {
    // eight connected grid so use
    return 0;
}

void PathPlanner::resetMap() {

}

void PathPlanner::resetGraph() {

}

bool PathPlanner::isGoal(double x, double y) {


}
