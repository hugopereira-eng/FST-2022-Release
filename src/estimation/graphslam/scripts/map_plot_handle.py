#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from common_msgs.msg import ConeDetections
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

import time
import math
import numpy as np

class MapPlotHandle:

    def __init__(self):
        
        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []
        self.blue_cones_real = []
        self.yellow_cones_real = []
        self.orange_cones_real = []
        self.distance = 0
        self.dx = 0
        self.lastDelta = rospy.Time.now().to_sec()
        self.lastPose = 0
        self.error_blue = []
        self.error_yellow = []
        self.error_orange = []
        self.firstUpdate = True
        self.lastdist = 0
        self.lasterrb = 0
        self.lasterry = 0
        self.lasterro = 0

        self.blues_real = [[4.44, 5942.230], 
        [7.73, 6147.690],
        [11.08, 6349.700],
        [13.69, 63.900],
        [16.05, 6383.700],
        [18.87, 6301.850],
        [22.43, 6263.355],
        [25.99, 6276.250],
        [29.16, 6334.590],
        [32.08, 6393.770],
        [34.38, 75.980],
        [35.09, 177.325],
        [34.50, 271.775],
        [32.11, 349.570],
        [28.99, 416.355],
        [26.16, 485.620],
        [25.36, 607.290],
        [25.06, 751.295],
        [25.02, 875.840],
        [23.72, 979.645],
        [21.82, 1045.950],
        [19.14, 1087.750],
        [16.52, 1138.445],
        [15.83, 1311.960],
        [16.33, 1481.170],
        [15.67, 1702.060,],
        [14.68, 1929.325],
        [14.13, 2133.875],
        [16.81, 2240.790],
        [19.86, 2284.730],
        [23.23, 2276.685],
        [26.52, 2274.155],
        [30.22, 2284.305],
        [32.04, 2339.885],
        [33.59, 2432.025],
        [34.21, 2514.065],
        [32.97, 2588.895],
        [30.51, 2683.200],
        [27.98, 2759.650],
        [26.64, 2833.050],
        [27.19, 2929.170],
        [27.05, 3057.660],
        [25.76, 3187.495],
        [23.67, 3299.595],
        [21.21, 3406.725],
        [17.74, 3492.305],
        [14.02, 3545.110],
        [10.53, 3427.040],
        [7.59, 3372.895],
        [4.32, 3572.295]]

        self.yellows_real =  [[4.51, 528.425], 
        [7.74, 317.620],
        [11.32, 384.325],
        [14.82, 381.63],
        [17.40, 242.985],
        [20.10, 114.130],
        [23.24, 57.635],
        [26.42, 50.035],
        [29.63, 96.990],
        [30.51, 190.240],
        [28.96, 270.675],
        [25.52, 312.335],
        [22.23, 374.105],
        [21.57, 521.120],
        [21.42, 665.165],
        [20.78, 808.780],
        [18.55, 874.130],
        [15.5, 872.385],
        [12.69, 919.900],
        [11.27, 1150.145],
        [12.23, 1420.090],
        [11.80, 1688.790],
        [10.70, 1923.655],
        [10.07, 2197.510],
        [11.93, 2404.865],
        [15.07, 2481.945],
        [18.16, 2503.875],
        [21.52, 2487.655],
        [23.86, 2447.615],
        [26.27, 2420.990],
        [28.65, 2438.305],
        [29.09, 2517.545],
        [27.21, 2599.010],
        [24.38, 2651.310],
        [22.27, 2745.805],
        [22.92, 2895.085],
        [23.05, 3004.830],
        [21.89, 3120.145],
        [19.48, 3222.680],
        [17.03, 3275.840],
        [14.16, 3247.540],
        [11.50, 3081.430],
        [9.15, 2855.615],
        [6.66, 2743.660],
        [4.09, 2616.175]]

        self.oranges_real = [[2.15, 1427.620],
                            [2.14, 1843.300],
                            [2.10, 5039.260],
                            [2.11, 4600.490]]
        # Publisher
        self.pubMapMarkers = rospy.Publisher('/estimation/map/landmarks', MarkerArray, queue_size = 1)

        # Subscribers
        self.subLandMarks = rospy.Subscriber('/estimation/slam/map', ConeDetections, self.landmarksCallback, queue_size=1)
        self.subOdometry = rospy.Subscriber('/estimation/slam/odometry', Odometry, self.odometryCallback , queue_size = 1)
        self.subLoopClosure = rospy.Subscriber('/estimation/slam/loop_closure', Bool, self.loopClosureCallback, queue_size = 1)

    def landmarksCallback(self, landmarks):
        self.blue_cones.clear()
        self.yellow_cones.clear()
        self.orange_cones.clear()

        for landmark in landmarks.cone_detections:
            if landmark.color == 0:
                self.blue_cones.append([landmark.position.x, landmark.position.y])
            elif landmark.color == 1:
                self.yellow_cones.append([landmark.position.x, landmark.position.y])
            elif landmark.color == 3:
                self.orange_cones.append([landmark.position.x, landmark.position.y])
    
        if(len(self.blue_cones) != 0):
            MapPlotHandle.plotError()
            
    def odometryCallback(self, odometry):
        
        if self.firstUpdate:
            dx = 0
            self.firstUpdate = False
        else:
            self.dx = odometry.header.stamp.to_sec() - self.lastDelta

        
        self.distance = self.distance + abs((math.hypot(odometry.pose.pose.position.x, odometry.pose.pose.position.y))-self.lastPose)
        self.lastDelta = odometry.header.stamp.to_sec()
        self.lastPose = abs(math.hypot(odometry.pose.pose.position.x, odometry.pose.pose.position.y))
        
    def loopClosureCallback(self, loopClosure):
        #if(loopClosure.data == True):
        pass   
    
    def publishMap(self):
        # Convert from (d, theta) to (x, y)
        cones = MarkerArray()
        cones.markers.clear()
        id = 0
        for blue in self.blues_real:
            theta = (2 * math.pi * blue[1]) / 6400
            pose = []
            pose.append(blue[0] * math.cos(theta))
            pose.append(-blue[0] * math.sin(theta))
            self.blue_cones_real.append([pose[0], pose[1]])
            marker = Marker()
            marker.id = id
            marker.action = marker.ADD
            marker.type = marker.SPHERE
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
            marker.lifetime = rospy.Duration(0)
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            cones.markers.append(marker)
            id += 1

        for yellow in self.yellows_real:
            theta = (2 * math.pi * yellow[1]) / 6400
            pose = []
            pose.append(yellow[0] * math.cos(theta))
            pose.append(-yellow[0] * math.sin(theta))
            self.yellow_cones_real.append([pose[0], pose[1]])
            marker = Marker()
            marker.id = id
            marker.action = marker.ADD
            marker.type = marker.SPHERE
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1
            marker.color.g = 1
            marker.color.b = 0
            marker.lifetime = rospy.Duration(0)
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            cones.markers.append(marker)
            id += 1
            
        for orange in self.oranges_real:
            theta = (2 * math.pi * orange[1]) / 6400
            pose = []
            pose.append(orange[0] * math.cos(theta))
            pose.append(-orange[0] * math.sin(theta))
            self.orange_cones_real.append([pose[0], pose[1]])
            marker = Marker()
            marker.id = id
            marker.action = marker.ADD
            marker.type = marker.SPHERE
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1
            marker.color.g = 0.65
            marker.color.b = 0
            marker.lifetime = rospy.Duration(0)
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            cones.markers.append(marker)
            id += 1

        self.pubMapMarkers.publish(cones)

    def plotError(self):
        acc_dist_blue = 0
        max_dist = 0
        for slam_cone in self.blue_cones:
            min_dist = 1000
            for track_cone in self.blue_cones_real:
                dist = math.sqrt((slam_cone[0] - track_cone[0])**2 + (slam_cone[1]-track_cone[1])**2) 
                if dist < min_dist:
                    min_dist = dist

            if min_dist > max_dist:
                max_dist = min_dist
            acc_dist_blue += min_dist

        self.error_blue.append([self.distance, acc_dist_blue/len(self.blue_cones)])
        print('RMSE BLUES', acc_dist_blue/len(self.blue_cones))

        acc_dist_yellow = 0
        max_dist = 0
        for slam_cone in self.yellow_cones:
            min_dist = 1000
            for track_cone in self.yellow_cones_real:
                dist = math.sqrt((slam_cone[0] - track_cone[0])**2 + (slam_cone[1]-track_cone[1])**2) 
                if dist < min_dist:
                    min_dist = dist
            
            if min_dist > max_dist:
                max_dist = min_dist
            acc_dist_yellow += min_dist

        self.error_yellow.append([self.distance, acc_dist_yellow/len(self.yellow_cones)])       
        print('RMSE Yellow', acc_dist_yellow/len(self.yellow_cones))

        acc_dist_orange = 0
        max_dist = 0
        for slam_cone in self.orange_cones:
            min_dist = 1000
            for track_cone in self.orange_cones_real:
                dist = math.sqrt((slam_cone[0] - track_cone[0])**2 + (slam_cone[1]-track_cone[1])**2) 
                if dist < min_dist:
                    min_dist = dist
            
            if min_dist > max_dist:
                max_dist = min_dist
            acc_dist_orange += min_dist
        
        self.error_orange.append([self.distance, acc_dist_orange/len(self.orange_cones)])
        print('RMSE Orange', acc_dist_orange/len(self.orange_cones))

        err_blue = acc_dist_blue/len(self.blue_cones)
        err_yellow = acc_dist_yellow/len(self.yellow_cones)
        err_orange = acc_dist_orange/len(self.orange_cones)
        
        # for i in range(len(self.error_blue)):
        plt.plot([self.lastdist, self.distance], [self.lasterrb, err_blue], '#3399FF')
        plt.plot([self.lastdist, self.distance], [self.lasterry, err_yellow], '#FFF300')
        plt.plot([self.lastdist, self.distance], [self.lasterro, err_orange], '#FFA500')
        
            # plt.plot(self.error_yellow[:, 0], self.error_yellow[:, 1])
            # plt.plot(self.error_orange[:, 0], self.error_orange[:, 1])
        plt.draw()
        self.lastdist = self.distance
        self.lasterrb = err_blue
        self.lasterry = err_yellow
        self.lasterro = err_orange

        

if __name__ == '__main__':
    rospy.init_node('MapPlotHandle')
    MapPlotHandle = MapPlotHandle()
    
    # while not rospy.is_shutdown():
    MapPlotHandle.publishMap()
    plt.cla()
    plt.pause(0.00001)
    plt.show()
    rospy.spin()