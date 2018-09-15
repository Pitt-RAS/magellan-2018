#! /usr/bin/env python
import sys
import rospy

from magellan_core.msg import WaypointStamped

def _go_to_goal(goal, pub):
    waypoint = WaypointStamped()
    waypoint.waypoint.goal.position.x = goal[0]
    waypoint.waypoint.goal.position.y = goal[1]
    pub.publish(waypoint)

def _cone_to_target(cones):
    return None

def _goal_done(goal):
    return False

if __name__ == '__main__':
    rospy.init_node('magellan_gameplay')

    pub = rospy.Publisher('/waypoint',
                          WaypointStamped,
                          queue_size=5)

    goals = [(1, 2), (3, 4)]
    cones = [(2, 2)]

    for goal in goals:
        cone = _cone_to_target(cones)
        if cone is not None:
            _next_goal = cone
        else:
            _next_goal = goal

        _go_to_goal(_next_goal, pub)

        while not _goal_done(_next_goal) and not rospy.is_shutdown():
            rospy.sleep(.05)

