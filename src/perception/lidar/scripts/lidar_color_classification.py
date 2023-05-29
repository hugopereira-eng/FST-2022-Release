#!/usr/bin/env python3

import torch
import numpy as np
import rospy
import cv2
from color_net import ColorNet
from math import sqrt
from common_msgs.msg import ConeDetections
class ColorClassification():

    def __init__(self):
        cuda = torch.cuda.is_available()
        self.device = torch.device('cuda:0' if cuda else 'cpu')
        self.pipeline = rospy.get_param('/perception/pipeline')  # Get pipeline parameter
        if self.pipeline == 0:
            self.modeFilePath = rospy.get_param('/lidar_color_classification/thesis_pipeline_weights_path')
        elif self.pipeline == 1:
            self.modeFilePath = rospy.get_param('/lidar_color_classification/darknet_pipeline_weights_path')
        self.model = ColorNet()
        self.model.load_state_dict(torch.load(self.modeFilePath, map_location = self.device).get('model'))
        self.model = self.model.to(self.device) # Run on GPU
        self.model.eval()

    # #
    #	Name: predictColor.
    #	Description: Loops through the image matrix and, if the correspondent cluster is less than 6m distance, send it to the model, which returns a prediction with color and prob.
    #	Inputs: coneMsg from the subscribers.
    #	Output: Prediction of the cluster's color and correspondent probability.
    # #
    def predictColor(self, coneMsg):
        coneArray = ConeDetections()
        if self.pipeline == 1:
            coneArray = coneMsg
        for i in range(0, len(coneMsg.matrix)):
            if sqrt(coneMsg.cone_detections[i].position.x**2 + coneMsg.cone_detections[i].position.y**2) < 4:
                image = self.loadImage(coneMsg.matrix[i].data, coneMsg.matrix[i].step)
                if cv2.countNonZero(image) > 10 :
                    # tensorImage = torch.from_numpy(image).type('torch.FloatTensor').view(-1,1,32,32)
                    # TO RUN ON CPU: UNCOMMENT PREVIOUS LINE AND COMMENT THE NEXT TWO LINES
                    tensorImage = torch.from_numpy(image).type('torch.cuda.FloatTensor').view(-1,1,32,32)
                    tensorImage.to(self.device)
                    output = self.model(tensorImage)
                    coneClass = self.decodeOutput(output)
                    self.assignColor(coneMsg.cone_detections[i], coneClass, output)
                else:
                    coneMsg.cone_detections[i].color = 4
    
                coneArray.cone_detections.append(coneMsg.cone_detections[i])
            elif self.pipeline == 1:
                coneMsg.cone_detections[i].color = 4
                coneArray.cone_detections.append(coneMsg.cone_detections[i])
        coneArray.header = coneMsg.header
        return coneArray

    # #
    #	Name: assignColor.
    #	Description: Assign color and probability of the prediction
    #	Inputs: coneMsg of the correspondent cone that is being evaluated and the predicted class.
    #	Output: update of the cone with color and correspondent probability.
    # #
    def assignColor(self, cone, coneClass, output):
        cone.color = coneClass
        cone.probability = np.amax(output.cpu().data.numpy())
    
    # #
    #	Name: loadImage.
    #	Description: Loads image from matrix
    #	Inputs: matrix of the correspondent cone that is being evaluated and step.
    #	Output: frame with the correspondent image.
    # #
    def loadImage(self, image, step):
        frame = np.zeros((step, step))
        for i in range(0,len(image)):
            j = i // step
            k = i % step
            frame[j,k] = image[i]
        return frame

    # #
    #	Name: decodeOutput.
    #	Description: Assignes the correct class to the output
    #	Inputs: NN output.
    #	Output: correct class.
    # #
    def decodeOutput(self, output):
        maxOut = np.amax(output.cpu().data.numpy())
        if maxOut <= 0.8:
            labelNum = 4        # Unkown
        else:
            outMax = np.argmax(output.cpu().data.numpy())
            if outMax == 0:
                labelNum = 0    # Blue
            elif outMax == 1:
                labelNum = 1    # Yellow
            else:
                labelNum = 4    # Unknown
        return labelNum
