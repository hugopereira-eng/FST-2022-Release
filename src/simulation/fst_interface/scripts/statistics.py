# AMZ-Driverless
# Copyright (c) 2018 Authors:
#   - Juraj Kabzan <kabzanj@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# ROS Include
import rospy
# Numpy
import numpy as np
# Msgs
from fssim_common.msg import State
# Geometry import
from shapely.geometry import Point
# YAML
import yaml
# System import
import os


def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)


# Return true if line segments AB and CD intersect
def intersect(A, B, C, D):
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def to_point(odom):
    return Point(odom.x, odom.y)


class LapStatistic:

    def __init__(self, folder):
        
        self.mission = ""
        self.mission_finished = False

        self.last_state = State()

        self.start_A = Point(0.0, 2.0)
        self.start_B = Point(0.0, -2.0)

        self.end_A = Point(0.0,0.0)
        self.end_B = Point(0.0,0.0)

        self.lap_count = 0
        self.vel_avg = 0

        self.rosbag_name = ""
        self.state_received = False

        self.starting_time = 0.0
        self.res_go_time = 0.0
        self.lap_time = []
        self.position_rmse = [0]
        self.cones_rmse = 0.0
        self.cones_hit = []
        self.centerline_rmse = []

        self.fst_laps = 0


        if folder is None:
            self.report_file_name = None
        else:
            if not os.path.isdir(folder):
                os.makedirs(folder)
            self.report_file_name = folder + '/fst_output.yaml'
        

    def is_mission_finnished(self):
        if self.mission == 'trackdrive':
            rospy.logwarn("Lap Count: %i, speed: %f", self.lap_count, self.last_state.vx)
            return self.lap_count == 11 and self.last_state.vx <= 1.5
        elif self.mission == 'acceleration':
            rospy.logwarn("State x: %f", self.last_state.x)
            return self.last_state.x > 76 and self.last_state.x < 120 and len(self.lap_time) is not 0
        elif self.mission == 'skidpad':
            rospy.logwarn("State x: %f, Laps: %i", self.last_state.x, self.lap_count)
            return self.last_state.x > 4 and self.last_state.x < 20 and self.lap_count == 5
        elif self.mission == 'autocross':
            rospy.logwarn("Lap Count: %i, speed: %f", self.lap_count, self.last_state.vx)
            return self.lap_count == 2 and self.last_state.vx <= 1.5

    def update_state(self, state):
        self.state_received = True
        flag = False
        if self.mission == 'trackdrive' or self.mission == 'skidpad' or self.mission == 'autocross':
            cross_line = intersect(self.start_A, self.start_B, to_point(self.last_state), to_point(state))
            if cross_line:
                self.lap_count = self.lap_count + 1
                if self.lap_count == 1:
                    self.starting_time = rospy.get_rostime().to_sec()
                    self.res_go_time = rospy.get_rostime().to_sec()

                else:
                    current_time  = rospy.get_rostime().to_sec()
                    self.lap_time.append(current_time - self.starting_time)
                    self.starting_time = current_time
                    rospy.logwarn("LAP Time: %f", self.lap_time[-1])
                rospy.logwarn("LAP: %i", self.lap_count)
                flag = True
        elif self.mission == 'acceleration':
            cross_line_start = intersect(self.start_A, self.start_B, to_point(self.last_state), to_point(state))
            cross_line_end = intersect(self.end_A, self.end_B, to_point(self.last_state), to_point(state))
            if cross_line_start:
                rospy.logwarn("Starting  to measure")
                self.starting_time = rospy.get_rostime().to_sec()
                self.res_go_time = rospy.get_rostime().to_sec()
            if cross_line_end:
                self.lap_time.append(rospy.get_rostime().to_sec() - self.starting_time)
                rospy.logwarn("STOP  stopwatch wioth time: %f", self.lap_time[-1])
        vel = np.sqrt(state.vx ** 2 + state.vy ** 2)
        if self.vel_avg == 0:
            self.vel_avg = vel
        else:
            self.vel_avg = (vel + self.vel_avg) / 2.0

        self.last_state = state

        return flag

    def get_rosbag_name(self, folder, sim_id):
        if folder is None:
            return None
        else:
            self.rosbag_name = str(sim_id) + '.bag'
            return folder + '/' + self.rosbag_name

    def get_duration(self):
        return 0.0 if self.res_go_time == 0.0 else rospy.get_rostime().to_sec() - self.res_go_time

    def get_statistics(self, id, repetition_info):
        name = 'Run ' + str(id)
        repetition_parameters = repetition_info['parameters'] if not None else []

        rospy.logwarn("Discipline successful: %i, mission_finnished: %i", self.is_mission_finnished(), self.mission_finished)
        repetition = {name: {'duration': self.get_duration(),
                             'pass': self.is_mission_finnished() and self.mission_finished,
                             'bag': self.rosbag_name,
                             'parameters': repetition_parameters,
                             'results': {
                                 'laps FSSIM': len(self.lap_time), 
                                 'laps FST': self.fst_laps, 
                                 'lap time': self.lap_time,
                                 'event time': sum(self.lap_time),
                                 'mission': self.mission,
                                 'position_rmse': self.average(self.position_rmse),
                                 'cones_rmse': self.cones_rmse,
                                 'cones_hit': self.cones_hit,
                                 'centerline_rmse': self.average(self.centerline_rmse)
                             },
                            }
                      }
        return repetition

    def get_best_configuration(self, results, metrics_to_check):
        best_results = {}
        for metric in metrics_to_check:
            best_repetition = []
            best_value = float("inf")
            for repetition_id, repetition in results["repetitions"].iteritems():
                metric_value = self.get_metric_value(repetition['results'][metric], metric)
                if metric_value != None and repetition['pass'] and metric_value <= best_value:
                    if metric_value < best_value:
                        best_repetition = []      
                    best_repetition.append(repetition_id)
                    best_value = metric_value
            best_results[metric] = best_repetition
        results["best configuration"] = best_results
        return results
    
    def average(self, list):
        return sum(list)/len(list)
    
    def get_metric_value(self, metric_value, metric):
        if metric == 'cones_hit':
            return sum(metric_value)
        elif metric == 'lap time':
            return None if metric_value == [] else min(metric_value)
        else:
            return metric_value
        
    def write_report(self, id, repetitions_info):
        if self.report_file_name is not None:
            with open(self.report_file_name, 'r+') as yamlfile:
                report_yaml = yaml.load(yamlfile)

                if report_yaml is None:
                    report_yaml = {"name": "default_name", "repetition": {}}

                repetitions = report_yaml['repetitions']
                if repetitions is None:
                    report_yaml['repetitions'] = self.get_statistics(id, repetitions_info[id])
                else:
                    report_yaml["repetitions"].update(self.get_statistics(id, repetitions_info[id]))
            
            #Check if this is the last repetition
            if id == len(repetitions_info) - 1 and repetitions_info[id]["parameters"] is not None:
                report_yaml = self.get_best_configuration(report_yaml, ['position_rmse', 'cones_rmse', 'cones_hit', 'lap time', 'event time','centerline_rmse'])
            
            with open(self.report_file_name, 'w+') as yamlfile:
                yaml.dump(report_yaml, yamlfile, default_flow_style = False)  # Also note the safe_dump