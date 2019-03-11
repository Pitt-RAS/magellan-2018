#include "planner.h"

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

static const double SQRT2_MINUS_ONE = 0.4142135624;
static const double SQRT2 = 1.4142135624;

using namespace MagellanPlanner;

PathPlanner::PathPlanner() {
    resetMap();
    resetGraph();
}

Path PathPlanner::plan(Point start, Point goal) {
    Path p;
    return p;
}

double PathPlanner::getHeuristic(double x, double y) {
}

void PathPlanner::resetMap() {

}

void PathPlanner::resetGraph() {

}
