#!/usr/bin/env python3

import rospy
from svm_path_planner import SvmPathPlanner
from common_msgs.msg import ConeDetections
from nav_msgs.msg import Path
import time

class SvmPathPlannerHandle:

    def __init__(self):
        self.SvmPathPlanner = SvmPathPlanner()
        # Subscribers
        rospy.Subscriber('/estimation/slam/map', ConeDetections, self.callback, queue_size = 1)

        # Publisher
        self._pubCenterLine = rospy.Publisher('/estimation/svm/centerline', Path, queue_size = 1)
        self._pubLeftBoundary = rospy.Publisher('/estimation/svm/leftBoundary', Path, queue_size = 1)
        self._pubRightBoundary = rospy.Publisher('/estimation/svm/rightBoundary', Path, queue_size = 1)

    def callback(self, cones):
        if len(cones.cone_detections) > 0:
            paths = self.SvmPathPlanner.generateCenterLine(cones)
            self._pubCenterLine.publish(paths[1])
            self._pubLeftBoundary.publish(paths[0])
            self._pubRightBoundary.publish(paths[2])
        
if __name__ == '__main__':
    rospy.init_node('SvmPathPlanner')
    svmPathPlannerHandle = SvmPathPlannerHandle()
    rospy.spin()