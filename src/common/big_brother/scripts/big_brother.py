#!/usr/bin/env python3

import rospy
import yaml
import base64
import rosnode
import rostopic
import os
import socket
import pickle
import numpy as np
import cv2 as cv
from threading import Thread
from common_msgs.msg import *
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, Bool


BUFFER_SIZE = 50000

SERVER_IP = ""
CAR_IP = ""


######################### AUXILIAR FUNCTIONS #########################


def sendData(sock, port, data):
    # send data to given address, breaking into multiple packets if needed

    global BUFFER_SIZE

    data = pickle.dumps(data)
    size = len(data)
    beg = 0
    end = BUFFER_SIZE if size > BUFFER_SIZE else size

    while size > 0:
        sock.sendto(data[beg:end], (SERVER_IP, port))
        size -= BUFFER_SIZE if size > BUFFER_SIZE else size
        beg += BUFFER_SIZE
        end += BUFFER_SIZE if size > BUFFER_SIZE else size


def receiveData(sock):
    # receive multiple packets of the same data

    global BUFFER_SIZE

    data = []
    while True:
        part = sock.recv(BUFFER_SIZE)
        data.append(part)
        if len(part) < BUFFER_SIZE:
            break
    return b''.join(data)


def image2jpeg(image):

    # encode frame as jpeg
    frame = cv.imencode('.jpg', image)[1].tobytes()

    # encode frame in base64 representation and remove utf-8 encoding
    frame = base64.b64encode(frame).decode('utf-8')
    return "data:image/jpeg;base64,{}".format(frame)



############################################################################################
######################################## MAIN CLASS ########################################
############################################################################################


class BigBrother():
    # because Big Brother is watching you |O.O|

    def __init__(self):

        # ROS info variables
        self.nodes = []
        self.topics = []
        self.frequencies = []
        self.rosTopicHz = []    # list of ROSTopicHz instances, one for each topic, which
                                # have a callback and save info to calculate topic frequency

        # AS info variables
        self.asTopics = {       # dictionary of the type [topic: RosTopicHz instance (later substituted if topic is being monitored)]
            "/perception/data_association/cone_detections": rostopic.ROSTopicHz(1),
            "/control/controller/control_cmd": rostopic.ROSTopicHz(1),
            "/common/mission_tracker/control_monitoring": rostopic.ROSTopicHz(1),
            "/common/mission_tracker/mission_finished": rostopic.ROSTopicHz(1),
            "/common/mission_tracker/lap_counter": rostopic.ROSTopicHz(1),
            "/common/res_state": rostopic.ROSTopicHz(1),
            "/missionTopic": rostopic.ROSTopicHz(1),
            "/common/can_sniffer/sta_position": rostopic.ROSTopicHz(1),
            "/estimation/slam/loop_closure": rostopic.ROSTopicHz(1),
            "/control/controller/point_to_follow": rostopic.ROSTopicHz(1),
            "/estimation/state_estimation/velocity": rostopic.ROSTopicHz(1),
            "/estimation/slam/map": rostopic.ROSTopicHz(1),
            "/common/pierre/visualization": rostopic.ROSTopicHz(1)    # plus image topic
            # "/arena_camera_node/image_raw": rostopic.ROSTopicHz(1)    # plus image topic
        }
        self.cones = {
            "blue": 0,
            "yellow": 0,
            "small-orange": 0,
            "big-orange": 0,
            "unknown": 0
        }
        self.throttle = .0
        self.steering = .0
        self.controlOn = False
        self.requireBraking = False
        self.mission = 0
        self.missionFinished = False
        self.lap = 0
        self.resEmergency = False
        self.resButtonPressed = False
        self.staPositionActual = 0.0
        self.steeringDeltaSTA = 0.0
        self.steeringDeltaController = 0.0
        self.steeringDeltaDemands = 0.0
        self.slamLoopClosure = False
        self.pointToFollowDistance = 0.0
        self.speedReference = 0.0
        self.velocityX = 0.0
        self.velocityY = 0.0
        self.mapCones = []

        # image variables
        self.image = None
        self.newImage = None

        # rosbag variables and threads
        self.rosbagData = None
        self.recording = False
        self.threadRosbag = Thread(target=self.receiveRosbagData, daemon=True)
        self.threadRecord = None
        
        # read config file (IPs, nodes and topics)
        self.readConfig()

        # initialize ROSTopicHz instances
        self.rosTopicHz = [rostopic.ROSTopicHz(100) for topic in self.topics]

        # sockets to communicate with server
        self.sockROSInfo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockASInfo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockImage = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockRosbag = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockRosbag.bind((CAR_IP, 3004))

        self.subscribeToTopics()

        self.threadRosbag.start()


    ####################### CALLBACK FUNCTIONS #######################
    

    def imageCallback(self, image, hz):
        
        # check if image is empty
        if image.data is [] or image.height == 0 or image.width == 0:
            return

        # call ROSTopicHz callback
        hz.callback_hz(image)

        # convert image to numpy array
        self.image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)


    def coneDetectionCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # reset cone count
        for color in self.cones.keys():
            self.cones[color] = 0

        # count cones of each color
        for cone in msg.cone_detections:
            if cone.color == 0:
                self.cones["blue"] += 1
            elif cone.color == 1:
                self.cones["yellow"] += 1
            elif cone.color == 2:
                self.cones["small-orange"] += 1
            elif cone.color == 3:
                self.cones["big-orange"] += 1
            else:
                self.cones["unknown"] += 1


    def controlCmdCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.throttle = round(msg.throttle, 3)
        self.steering = round(msg.steering_angle, 3)


    def controlMonitoringCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.controlOn = msg.control_on
        self.requireBraking = msg.require_braking


    def missionFinishedCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.missionFinished = msg.finished


    def lapCounterCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.lap = msg.data


    def resStateCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.resEmergency = msg.emergency
        self.resButtonPressed = msg.push_button
    

    def missionCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.mission = msg.mission
    

    def staPositionCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.staPositionActual = msg.positionActual
        self.steeringDeltaSTA = msg.deltaSTA
        self.steeringDeltaController = msg.deltaController
        self.steeringDeltaDemands = abs(msg.positionDemandController - msg.positionDemandSTA)
    

    def slamCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.slamLoopClosure = msg.data
    

    def pointToFollowCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.pointToFollowDistance = round(msg.distance,2)
        self.speedReference = round(msg.speed_ref,2)
    

    def velocityCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.velocityX = round(msg.velocity.x, 2)
        self.velocityY = round(msg.velocity.y, 2)
    

    def mapCallback(self, msg, hz):
        
        # call ROSTopicHz callback
        hz.callback_hz(msg)

        # save data
        self.mapCones = [((cone.position.x, cone.position.y), cone.color) for cone in msg.cone_detections]

    
    #################### AUXILIAR CLASS FUNCTIONS ####################


    def readConfig(self):

        global SERVER_IP, CAR_IP

        # Read config file
        try:
            file = open(os.path.dirname(__file__) + "/../config/config.yaml", 'r')
            doc = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(exc)
        finally:
            file.close()
        
        # initialize variables
        CAR_IP = doc["car_ip"]
        SERVER_IP = doc["server_ip"]
        self.nodes = doc["nodes"]
        self.topics = doc["topics"]


    def subscribeToTopics(self):
        # if an AS topic is on the config file to be monitored then its ROSTopicHz(1) instance in the self.asTopics
        # dictionary is replaced by the corresponding ROSTopicHz(100) instance in the self.rosTopicHz array
        
        # topics to be monitored
        for topic, hz in zip(self.topics, self.rosTopicHz):
            if topic in self.asTopics.keys():
                self.asTopics[topic] = hz   # replace ROSTopicHz instance
            else:
                rospy.Subscriber(topic, rospy.AnyMsg, hz.callback_hz)

        # AS info topics
        rospy.Subscriber("/perception/data_association/cone_detections", ConeDetections,\
            self.coneDetectionCallback, callback_args=self.asTopics["/perception/data_association/cone_detections"])
        rospy.Subscriber("/control/controller/control_cmd", ControlCmd,\
            self.controlCmdCallback, callback_args=self.asTopics["/control/controller/control_cmd"])
        rospy.Subscriber("/common/mission_tracker/control_monitoring", ControlMonitoring,\
            self.controlMonitoringCallback, callback_args=self.asTopics["/common/mission_tracker/control_monitoring"])
        rospy.Subscriber("/common/mission_tracker/mission_finished", Mission,\
            self.missionFinishedCallback, callback_args=self.asTopics["/common/mission_tracker/mission_finished"])
        rospy.Subscriber("/common/mission_tracker/lap_counter", Int16,\
            self.lapCounterCallback, callback_args=self.asTopics["/common/mission_tracker/lap_counter"])
        rospy.Subscriber("/common/res_state", RES,\
            self.resStateCallback, callback_args=self.asTopics["/common/res_state"])
        rospy.Subscriber("/missionTopic", Mission,\
            self.missionCallback, callback_args=self.asTopics["/missionTopic"])
        rospy.Subscriber("/common/can_sniffer/sta_position", StaPositionInfo,\
            self.staPositionCallback, callback_args=self.asTopics["/common/can_sniffer/sta_position"])
        rospy.Subscriber("/estimation/slam/loop_closure", Bool,\
            self.slamCallback, callback_args=self.asTopics["/estimation/slam/loop_closure"])
        rospy.Subscriber("/control/controller/point_to_follow", PointToFollow,\
            self.pointToFollowCallback, callback_args=self.asTopics["/control/controller/point_to_follow"])
        rospy.Subscriber("/estimation/state_estimation/velocity", CarVelocity,\
            self.velocityCallback, callback_args=self.asTopics["/estimation/state_estimation/velocity"])
        rospy.Subscriber("/estimation/slam/map", ConeDetections,\
            self.mapCallback, callback_args=self.asTopics["/estimation/slam/map"])

        # image topic
        rospy.Subscriber("/common/pierre/visualization", Image,\
            self.imageCallback, callback_args=self.asTopics["/common/pierre/visualization"])
        # rospy.Subscriber("/arena_camera_node/image_raw", Image,\
        #     self.imageCallback, callback_args=self.asTopics["/arena_camera_node/image_raw"])


    def calculateFrequencies(self):
        # uses info from each ROSTopicHz instance to calculate the frequency of each topic

        rates = []

        for hz in self.rosTopicHz:
            n = len(hz.times)
            mean = sum(hz.times) / n if n != 0 else 0
            rates += [round(1./mean if mean > 0. else 0)]

        return rates


    def buildRosInfo(self):
        # build ROS info:
        #   - list of booleans, one for each node (True = active, False = inactive)
        #   - list of frequencies, one for each monitored topic

        activeNodes = rosnode.get_node_names()
        info = [list(map(lambda node: any(node in nd for nd in activeNodes), self.nodes)), self.frequencies]
        
        return info
    

    def buildAsInfo(self):
        # build AS info from AS topics

        info = list(self.cones.values()) +\
            [self.throttle,
            self.steering,
            self.controlOn,
            self.requireBraking,
            self.mission,
            self.missionFinished,
            self.lap,
            self.resEmergency,
            self.resButtonPressed,
            self.staPositionActual,
            self.steeringDeltaSTA,
            self.steeringDeltaController,
            self.steeringDeltaDemands,
            self.slamLoopClosure,
            self.pointToFollowDistance,
            self.speedReference,
            self.velocityX,
            self.velocityY,
            self.mapCones]
        
        return info
    

    def closeSockets(self):
        
        self.sockROSInfo.close()
        self.sockASInfo.close()
        self.sockImage.close()
        self.sockRosbag.close()


    ######################## ROSBAG FUNCTIONS ########################

    def receiveRosbagData(self):
        # constantly receive rosbag data from server (record, name, topics)

        while True:
            data = receiveData(self.sockRosbag)
            self.rosbagData = pickle.loads(data)
    

    def handleRosbag(self):
        # start or stop rosbag record according to server rosbag data

        if self.rosbagData is None: return

        # start recording by starting recording thread
        if self.rosbagData["record"] and not self.recording:
            self.threadRecord = Thread(target=self.recordRosbag, daemon=True)
            self.threadRecord.start()
            self.recording = True

        # stop recording by killing recording node
        elif not self.rosbagData["record"] and self.recording:
            node = list(filter(lambda nd: "record" in nd, rosnode._sub_rosnode_listnodes().split()))
            if node != []:
                rosnode.kill_nodes(node)
            self.recording = False


    def recordRosbag(self):
        # execute rosbag record command

        topics_str = ""
        for topic in self.rosbagData["topics"]:
            topics_str += topic + " "
        # cmd = topics_str + "-O ~/rosbags/" + self.rosbagData["name"]
        cmd = topics_str + "-O ~/rosbags/" + self.rosbagData["name"] + " -b 4096"
        os.system("rosbag record " + cmd)



    #################### MAIN CLASS FUNCTION ####################
        

    def run(self):

        # run forever
        while not rospy.is_shutdown():

            # start or stop rosbag record
            self.handleRosbag()

            # ROS info
            self.frequencies = self.calculateFrequencies()
            sendData(self.sockROSInfo, 3001, self.buildRosInfo())   # send to receiver

            # AS info
            sendData(self.sockASInfo, 3002, self.buildAsInfo())     # send to receiver

            # image
            if self.image is not None:
                # image downsampling
                self.newImage = self.image[::6, ::6]
                # send to server
                sendData(self.sockImage, 3003, image2jpeg(self.newImage))

            rate.sleep()



############################################################################################


if __name__ == '__main__':

    rospy.init_node('bigBrother')
    rate = rospy.Rate(15)
    bigBrother = BigBrother()

    try:
        bigBrother.run()
    except rospy.ROSInterruptException:
        bigBrother.closeSockets()
