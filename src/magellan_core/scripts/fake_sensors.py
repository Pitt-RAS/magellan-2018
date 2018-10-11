#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PolygonStamped
from magellan_core.msg import (ObstacleStamped, ObstacleStampedArray)
from visualization_msgs.msg import Marker

class FakeObstacles(object):
    def __init__(self, obstacles):
        self._marker_pub = rospy.Publisher('/obstacle_markers',
                              Marker,
                              queue_size=5)

        self._obst_pub = rospy.Publisher('/obstacles',
                              ObstacleStampedArray,
                              queue_size=5)

        self._msg = ObstacleStampedArray()

        for name, values in obstacles.iteritems():
            _obst = ObstacleStamped()
            _obst.obst.x = values['x']
            _obst.obst.y = values['y']
            _obst.obst.width = values['width']
            _obst.obst.length = values['length']
            _obst.header.stamp = rospy.Time.now()

            self._msg.obstacles.append(_obst)

    def _publish_obstacles(self):
        self._obst_pub.publish(self._msg)

    def _publish_markers(self):
        pass

    def update(self):
        self._publish_obstacles()
        self._publish_markers()

if __name__ == '__main__':
    rospy.init_node('fake_sensors')

    # 20 hz update rate
    _rate = rospy.Rate(20)

    obsts = FakeObstacles(rospy.get_param('~fake_obstacles'))

    while not rospy.is_shutdown():
        obsts.update()
        _rate.sleep()

