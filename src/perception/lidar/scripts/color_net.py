#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
from collections import OrderedDict
class ColorNet(nn.Module):
    # Defining the Neural Network structure (layers)
    # #
    #	Name: __init__.
    #	Description: Neural Network initialization with the correspondent layers to be used.
    #	Inputs: none.
    #	Output: initilized NN.
    # #
    def __init__(self):
        super(ColorNet, self).__init__()
        self.features = nn.Sequential(OrderedDict([
                            ('conv1', nn.Conv2d(in_channels = 1, out_channels = 32, kernel_size = 7, padding = 3)),
                            ('relu1', nn.ReLU()),
                            ('maxPooling1', nn.MaxPool2d((2, 2))),
                            ('conv2', nn.Conv2d(in_channels = 32, out_channels = 64, kernel_size = 5, padding = 2)),
                            ('batchNorm2', nn.BatchNorm2d(64)),
                            ('relu2', nn.ReLU()),
                            ('maxPooling2', nn.MaxPool2d((2, 2))),
                            ('conv3', nn.Conv2d(in_channels = 64, out_channels = 128, kernel_size = 3, padding = 1)),
                            ('batchNorm3', nn.BatchNorm2d(128)),
                            ('relu3', nn.ReLU()),
                            ('maxPooling3', nn.MaxPool2d((2, 2))),
                            ('conv4', nn.Conv2d(in_channels = 128, out_channels = 256, kernel_size = 3, padding = 1)),
                            ('relu4', nn.ReLU())
                            ]))

        self.classifier = nn.Sequential(OrderedDict([
                            ('fc1', nn.Linear(4*4*256, 1024)),
                            ('relu1', nn.ReLU(True)),
                            ('dropout1', nn.Dropout()),
                            ('fc2', nn.Linear(1024, 256)),
                            ('relu2', nn.ReLU(True)),
                            ('dropout2', nn.Dropout()),
                            ('fc3', nn.Linear(256, 64)),
                            ('relu3', nn.ReLU(True)),
                            ('dropout3', nn.Dropout()),
                            ('fc4', nn.Linear(64, 16)),
                            ('relu4', nn.ReLU(True)),
                            ('dropout4', nn.Dropout()),
                            ('fc5', nn.Linear(16, 3)),
                            ]))

    # #
    #	Name: _initializeWeights.
    #	Description: Initializes the Neural Network weights
    #	Inputs: none.
    #	Output: weight initialization.
    # #
    def _initializeWeights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode = 'fan_out', nonlinearity = 'relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.constant_(m.bias, 0)

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1) 
        x = self.classifier(x)
        return F.softmax(x, dim = 1)

if  __name__=='__main__':
    ColorNet()
