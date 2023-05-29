#!/usr/bin/env python

#arguments
import argparse

#Python Import 
import os
import yaml
import rospy
import math
from statistics import LapStatistic

# Msgs
from fssim_common.msg import State, Track
from common_msgs.msg import Mission, Track as fst_track
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

import rospkg
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path

# Geometry import
from shapely.geometry import Point
import sys
class AutoTestHandle:
    
    def __init__(self, arg):

        self.output_folder = arg.output
        self.mission_id = arg.sim_id

        with open(args.config, 'r') as config_f:
            self.config = yaml.load(config_f)

        path_track_midpoints = rospkg.RosPack().get_path("fssim") + "/../fssim_gazebo/models/track/tracks_midpoints/"
        file_midpoints = path_track_midpoints + self.config['repetitions'][0]['track_name'][:-4] + ".yaml"
        with open(file_midpoints, 'r') as track_mid:
            self.track_midpoints = yaml.load(track_mid)["midpoints (x,y)"]

        rospy.logwarn("waiting for MISSION to be set")
        while not rospy.has_param('common/mission_selected'):
            pass
        rospy.logwarn("MISSION received")    
        self.mission = rospy.get_param('common/mission_selected')

        self.transform = TransformStamped()
        self.original_track = PointCloud2()
        self.cone_hit_flag = False
        self.cones_hit = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.loop_clouse = False
        self.centerline_error = []

        self.statistics = LapStatistic(self.output_folder)
        self.sub_state = rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.callback_state)
        self.sub_track = rospy.Subscriber('/fssim/track', Track, self.callback_track)
        self.sub_mission_finished = rospy.Subscriber('/common/mission_tracker/mission_finished', Mission, self.callback_mission_finished)

        self.sub_nr_laps = rospy.Subscriber('/common/mission_tracker/lap_counter', Int16, self.callback_nr_laps)

        self.sub_slam_state = rospy.Subscriber('/estimation/slam/cones_autotests', fst_track, self.callback_slam_cones)
        self.sub_slam_loop_closure = rospy.Subscriber('/estimation/slam/loop_closure', Int16, self.callback_loop_closure)
        self.sub_slam_odometry = rospy.Subscriber('/estimation/slam/odometry', Odometry, self.callback_slam_odometry)
        self.sub_original_track = rospy.Subscriber('/lidar/cones', PointCloud2, self.callback_original_track)
        self.sub_path_planner = rospy.Subscriber('/control/svm/centerline', Path, self.callback_path_planner)
        self.pub_transformed_observations = rospy.Publisher('/tf_cones', PointCloud2, queue_size = 1)

        self.statistics.mission = self.mission
        
        if self.mission == "skidpad":
            self.statistics.centerline_rmse.append(0)
        

    def write_report(self):
        try:
            self.config['repetitions'][0]['parameters']
        except KeyError:
            for repetition in self.config['repetitions']:
                repetition['parameters'] = None
        self.statistics.write_report(self.mission_id, self.config['repetitions'])

    def callback_state(self, data):
        if self.statistics.update_state(data):
            self.cone_hit_flag = True

    def callback_track(self, data):
        rospy.logwarn("Track was received in FST_statistics")
        self.track = data

        if len(data.tk_device_start) == 2:
            self.statistics.start_A = Point(data.tk_device_start[0].x, data.tk_device_start[0].y)
            self.statistics.start_B = Point(data.tk_device_start[1].x, data.tk_device_start[1].y)
        if len(data.tk_device_end) == 2:
            self.statistics.end_A = Point(data.tk_device_end[0].x, data.tk_device_end[0].y)
            self.statistics.end_B = Point(data.tk_device_end[1].x, data.tk_device_end[1].y)
        else:
            self.statistics.end_A = self.statistics.start_A
            self.statistics.end_B = self.statistics.start_B

    def callback_mission_finished(self, data):
        self.statistics.mission_finished = data.finished
    
    def callback_nr_laps(self, data):
        self.statistics.fst_laps = data.data

    def callback_slam_cones(self, data):
        #rospy.logwarn("Slam track received")
        self.slam_cones = data
    
    def callback_slam_odometry(self, data):
        # rospy.logwarn('FSSIM: ' + str(self.statistics.last_state.x) + ', '+ str(self.statistics.last_state.y))
        # rospy.logwarn('SLAM_: ' + str(data.pose.pose.position.x) + ', ' + str(data.pose.pose.position.y))

        distance = math.sqrt((self.statistics.last_state.x - data.pose.pose.position.x)**2 + (self.statistics.last_state.y - data.pose.pose.position.y)**2)
        self.statistics.position_rmse.append(distance)

    def callback_loop_closure(self, data):
        self.loop_clouse = True
        rospy.logwarn('Loop closure, calculating RMSE...')
        num_cones = len(self.slam_cones.cones_orange) + \
                    len(self.slam_cones.cones_left) + \
                    len(self.slam_cones.cones_right)

        rospy.logwarn('Slam saw ' + str(num_cones) + ' cones')

        acc_dist_orange = 0
        max_dist = 0
        for slam_cone in self.slam_cones.cones_orange:
            min_dist = 1000
            
            for track_cone in self.track.cones_orange + self.track.cones_orange_big:
                dist = math.sqrt((slam_cone.x - track_cone.x)**2 + (slam_cone.y-track_cone.y)**2) 
                if dist < min_dist:
                    min_dist = dist

            if min_dist > max_dist:
                max_dist = min_dist
            acc_dist_orange += min_dist
        
        rospy.logwarn('RMSE for orange cones: ' + str(acc_dist_orange/len(self.slam_cones.cones_orange)))
        rospy.logwarn('Maximum association distance for orange cones: ' + str(max_dist))

        acc_dist_left = 0
        max_dist = 0
        for slam_cone in self.slam_cones.cones_left:
            min_dist = 1000
            for track_cone in self.track.cones_left:
                dist = math.sqrt((slam_cone.x - track_cone.x)**2 + (slam_cone.y-track_cone.y)**2) 
                if dist < min_dist:
                    min_dist = dist

            if min_dist > max_dist:
                max_dist = min_dist
            acc_dist_left += min_dist
        
        rospy.logwarn('RMSE for left cones: ' + str(acc_dist_left/len(self.slam_cones.cones_left)))
        rospy.logwarn('Maximum association distance for left cones: ' + str(max_dist))

        acc_dist_right = 0
        max_dist = 0
        for slam_cone in self.slam_cones.cones_right:
            min_dist = 1000
            for track_cone in self.track.cones_right:
                dist = math.sqrt((slam_cone.x - track_cone.x)**2 + (slam_cone.y-track_cone.y)**2) 
                if dist < min_dist:
                    min_dist = dist
            
            if min_dist > max_dist:
                max_dist = min_dist
            acc_dist_right += min_dist
        
        rospy.logwarn('RMSE for right cones: ' + str(acc_dist_right/len(self.slam_cones.cones_right)))
        rospy.logwarn('Maximum association distance for right cones: ' + str(max_dist))

        rospy.logwarn('RMSE for all cones: ' + str((acc_dist_left + acc_dist_orange + acc_dist_right)/num_cones))
        self.statistics.cones_rmse = (acc_dist_left + acc_dist_orange + acc_dist_right)/num_cones

    
    def callback_original_track (self, data):
        if self.cone_hit_flag:
            rospy.logwarn("----- SAMPLING TRACK -----")
            
            try:
                self.transform = self.tf_buffer.lookup_transform('map', data.header.frame_id,
                                        data.header.stamp,
                                        rospy.Duration(5.0))
            except tf2.LookupException as ex:
                rospy.logwarn(ex)
                return
            except tf2.ExtrapolationException as ex:
                rospy.logwarn(ex)
                return

            self.cone_hit_flag = False
            cloud_out = do_transform_cloud(data, self.transform)
            self.pub_transformed_observations.publish(cloud_out)

            if self.original_track.width == 0:
                rospy.logwarn('-- Original track --')
                self.original_track = cloud_out
                return

            cones_hit = 0
            for original_cone in pc2.read_points(self.original_track):
                o_x = original_cone[0]
                o_y = original_cone[1]
                min = 1000
                for sampled_cone in pc2.read_points(cloud_out):
                    n_x = sampled_cone[0]
                    n_y = sampled_cone[1]
                    dist = math.hypot(o_x - n_x, o_y - n_y) 
                    if dist < min:
                        min = dist
                
                if min > 0.01:
                    cones_hit += 1
            
            rospy.logwarn("Cones hit total: " + str(cones_hit))
            self.cones_hit = cones_hit - self.cones_hit
            rospy.logwarn("New cones hit this lap: " + str(self.cones_hit))
            self.statistics.cones_hit.append(self.cones_hit)
            

    def callback_path_planner (self, data):
        if not self.loop_clouse:
            min_dist = float("inf")
            
            for midpoint in self.track_midpoints:
                dist = math.sqrt((self.statistics.last_state.x - midpoint[0])**2 + (self.statistics.last_state.y-midpoint[1])**2) 
                if dist < min_dist:
                    min_dist = dist
                    closest_midpoint = midpoint

            self.statistics.centerline_rmse.append(min_dist)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Information for statistics.')
    parser.add_argument("--config", dest = "config", metavar = "FILE", help = "Config YAML file")
    parser.add_argument("--id", dest = "sim_id", help = "Config ID in YAML file", type = int)
    parser.add_argument("--output", dest = "output", metavar = "FOLDER", help = "Output YAML file")

    args, unknown = parser.parse_known_args()
    args.output = os.path.abspath(args.output) if args.output is not None else None

    rospy.init_node('auto_test_handle')
    autoTestHandle = AutoTestHandle(args)
    rospy.on_shutdown(autoTestHandle.write_report)
    rospy.spin()