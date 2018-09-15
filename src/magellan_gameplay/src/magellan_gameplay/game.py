#! /usr/bin/env python
import sys
import rospy

from magellan_core.msg import WaypointStamped

def _go_to_goal(goal):
    pass

def _cone_to_target(cones):
    return None

def _goal_done(goal):
    return False

if __name__ == '__main__':
    rospy.init_node('magellan_gameplay')

    goals = [(1, 2), (3, 4)]
    cones = [(2, 2)]

    for goal in goals:
        cone = _cone_to_target(cones)
        if cone is not None:
            _next_goal = cone
        else:
            _next_goal = goal

        while not _goal_done(goal) and not rospy.is_shutdown():
            rospy.sleep(0.10)

