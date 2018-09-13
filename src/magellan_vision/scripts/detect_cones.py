#!/usr/bin/env python2
import rospy
import yolo
from geometry_msgs.msg import Point

class ConeDetectNode:
    def __init__(self):
        # Placeholder for the eventual image publisher
        self.image_stream_sub = rospy.Subscriber('rgb_image', Image, self._process_image)
        # TODO: make custom message to hold cone coordinates and possibly detection score (confidence)
        self.cone_loc_pub = rospy.Publisher('cone_loc', Point, queue_size=10)
        self.yolo = yolo.YOLO()
        self.coneLocations = []
        self.coneScores = []

    def _process_image(self, img):
        (boxes, scores, classes) = self.yolo.detect_image(img)
        for (box, score, objClass) in zip(boxes, scores, classes):
            self.coneLocations = []
            self.coneScores = []
            if objClass == 'cone':
                # TODO: change this to be the cone location
                self.coneLocations.append(Point(0))
                self.coneScores.append(score)

    def update(self):
        self.cone_loc_pub.publish(self.coneLocations[0])
    
rospy.init_node('cone_loc', anonymous = False)
rate = rospy.Rate(rospy.et_param('~rate', 10))
node = ConeDetectNode()
while not rospy.is_shutdown():
    node.update()
    rate.sleep()            
