#!/usr/bin/env python3

import rospy
from lidar_color_classification import ColorClassification
from common_msgs.msg import ConeDetections
import message_filters
from std_msgs.msg import Bool 

class ColorClassificationHandle:

    def __init__(self):
        self.lidar_color_classification = ColorClassification()
        # Get pipeline parameter
        self.pipeline = rospy.get_param('/perception/pipeline')
        # Subscribers
        if self.pipeline == 0:
            # Thesis pipeline
            rospy.Subscriber('/perception/sensor_fusion/colorNet_proposals', ConeDetections, self.ConeDetectionsCallback)
        elif self.pipeline == 1:
            # Darknet pipeline
            rospy.Subscriber('/perception/lidar/cluster_centroids', ConeDetections, self.ConeDetectionsCallback)
        # Publisher
        self._pubCone = rospy.Publisher('/perception/lidar/colorNet_cone_detections', ConeDetections, queue_size=1)
        self._colorClassActive = rospy.Publisher('/perception/lidar/color_classification_active', Bool, queue_size=1)
        
    def ConeDetectionsCallback(self, coneMsg):
        coneWColor = self.lidar_color_classification.predictColor(coneMsg)
        self._pubCone.publish(coneWColor)
        msg = Bool()
        msg.data = True
        self._colorClassActive.publish(msg)

    
if __name__ == '__main__':
    rospy.init_node('lidar_color_classification')
    rospy.Rate(10)
    colorClassificationHandle = ColorClassificationHandle()
    rospy.spin()
    
