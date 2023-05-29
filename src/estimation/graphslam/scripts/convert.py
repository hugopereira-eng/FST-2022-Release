import numpy as np
import math
import matplotlib.pyplot as plt
import sys
import yaml

# theta  = (2 * pi * theta) / 6400 
# pose(x, y) = (d * cos(theta), d * sin(theta))

# Starting Postion
start_pos = np.array([4.23, 3204.785])
theta = (2 * math.pi * start_pos[1]) / 6400
starting_pos = np.zeros(2)
starting_pos[0] = start_pos[0] * math.cos(theta)
starting_pos[1] = -start_pos[0] * math.sin(theta)

# Cone Position ground truth
blues_real = [[4.44, 5942.230], 
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

yellows_real = [[4.51, 528.425], 
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

oranges_real = [[2.15, 1427.620],
                        [2.14, 1843.300],
                        [2.10, 5039.260],
                        [2.11, 4600.490]]
                    

# Computed Track
yellows = [[3.673, -2.379],
        [7.092, -2.559],
        [16.596, -4.372],
        [10.227, -4.354],
        [13.453, -5.627],
        [19.616, -2.485],
        [22.826, -1.301],
        [23.970, -7.876],
        [27.579, -7.695],
        [20.373, -8.155],
        [29.243, -3.860],
        [25.953, -1.719],
        [29.389, -6.070],
        [18.221, -10.743],
        [9.887, -12.079],
        [13.845, -14.870],
        [16.396, -13.205],
        [11.590, -14.046],
        [1.797, -11.871],
        [4.491, -10.116],
        [-1.395, -11.454],
        [-5.824, -8.105],
        [-3.652, -9.934],
        [7.544, -10.011],
        [-8.683, -8.023],
        [-11.788, -9.326],
        [-16.973, -13.559],
        [-14.438, -10.988],
        [-18.670, 0.982],
        [-22.799, -14.996],
        [-18.016, -15.585],
        [-19.297, -17.690],
        [-21.291, -19.012],
        [-23.015, -17.711],
        [-21.014, -12.320],
        [-22.450, -4.102],
        [-21.504, -6.394],
        [-20.027, -9.292],
        [-21.604, -1.460],
        [-16.766, 1.612],
        [-14.092, 1.088],
        [-11.376, -0.883],
        [-8.714, -2.718],
        [-6.065, -2.763]]

blues = [[3.749, 1.746],
        [7.207, 1.668],
        [10.798, 0.347],
        [13.372, -1.132],
        [15.785, -0.021],
        [18.467, 1.425],
        [25.434, 2.697],
        [28.743, 1.373],
        [31.650, -0.303],
        [21.812, 2.546],
        [33.823, -3.190],
        [33.382, -8.070],
        [29.460, -11.314],
        [25.738, -11.896],
        [22.653, -12.391],
        [20.309, -14.535],
        [17.763, -16.986],
        [15.503, -19.040],
        [8.507, -16.733],
        [12.747, -19.481],
        [10.479, -18.658],
        [6.730, -14.751],
        [3.996, -15.030],
        [1.267, -15.833],
        [-2.002, -15.215],
        [-5.039, -13.494],
        [-7.395, -11.785],
        [-10.198, -13.110],
        [-12.764, -14.937],
        [-14.711, -17.695],
        [-21.508, -23.444],
        [-16.612, -20.263],
        [-19.094, -23.113],
        [-24.958, -22.274],
        [-26.937, -20.339],
        [-27.254, -18.027],
        [-25.177, -11.407],
        [-26.549, -14.228],
        [-23.124, 2.622],
        [-24.930, -9.050],
        [-26.138, -6.921],
        [-20.307, 4.534],
        [-26.396, -3.400],
        [-25.377, 0.012],
        [-16.576, 5.207],
        [-12.851, 4.849],
        [-7.386, 1.484],
        [-10.006, 2.608],
        [-4.069, 1.567]]

oranges = [[-0.667, -2.107],
        [-0.569, 1.985],
        [0.148, -2.222],
        [0.277, 1.893]]


# Convert from (d, theta) to (x, y)
blue_cones = []
for blue in blues_real:
    theta = (2 * math.pi * blue[1]) / 6400
    pose = []
    pose.append(blue[0] * math.cos(theta))
    pose.append(-blue[0] * math.sin(theta))
    blue_cones.append([pose[0], pose[1]])

yellow_cones = []
for yellow in yellows_real:
    theta = (2 * math.pi * yellow[1]) / 6400
    pose = []
    pose.append(yellow[0] * math.cos(theta))
    pose.append(-yellow[0] * math.sin(theta))
    yellow_cones.append([pose[0], pose[1]])

orange_cones = []
for orange in oranges_real:
    theta = (2 * math.pi * orange[1]) / 6400
    pose = []
    pose.append(orange[0] * math.cos(theta))
    pose.append(-orange[0] * math.sin(theta))
    orange_cones.append([pose[0], pose[1]])

# Save to .yaml file
#file = open('centralVN.yaml', 'w')
#file.write("blue_cones: \n")
#yaml.dump(blue_cones, file)
#file.write("yellow_cones: \n")
#yaml.dump(yellow_cones, file)
#file.write("orange_cones: \n")
#yaml.dump(orange_cones, file)
#file.close()

#Visualize RMSE

acc_dist_blue = 0
max_dist = 0
for slam_cone in blues:
    min_dist = 1000
    for track_cone in blue_cones:
        dist = math.sqrt((slam_cone[0] - track_cone[0])**2 + (slam_cone[1]-track_cone[1])**2) 
        if dist < min_dist:
            min_dist = dist

    if min_dist > max_dist:
        max_dist = min_dist
    acc_dist_blue += min_dist

print('RMSE for blue cones: ' + str(acc_dist_blue/len(blues)))
print('Maximum association distance for blue cones: ' + str(max_dist))

acc_dist_yellow = 0
max_dist = 0
for slam_cone in yellows:
    min_dist = 1000
    for track_cone in yellow_cones:
        dist = math.sqrt((slam_cone[0] - track_cone[0])**2 + (slam_cone[1]-track_cone[1])**2) 
        if dist < min_dist:
            min_dist = dist
    
    if min_dist > max_dist:
        max_dist = min_dist
    acc_dist_yellow += min_dist

print('RMSE for yellow cones: ' + str(acc_dist_yellow/len(yellows)))
print('Maximum association distance for yellow cones: ' + str(max_dist))

acc_dist_orange = 0
max_dist = 0
for slam_cone in oranges:
    min_dist = 1000
    for track_cone in orange_cones:
        dist = math.sqrt((slam_cone[0] - track_cone[0])**2 + (slam_cone[1]-track_cone[1])**2) 
        if dist < min_dist:
            min_dist = dist
    
    if min_dist > max_dist:
        max_dist = min_dist
    acc_dist_orange += min_dist

print('RMSE for orange cones: ' + str(acc_dist_orange/len(oranges)))
print('Maximum association distance for orange cones: ' + str(max_dist))


# Visualization Cone Positions

# color_yellow = np.full(len(yellow_cones), "#FFF300")
# color_blue = np.full(len(blue_cones), "#3399FF")
# color_orange = np.full(len(orange_cones), "#FFA500")
# color_green =  np.full(1, "#17A536")

# all_cones = np.concatenate((np.concatenate((yellow_cones, blue_cones)), orange_cones))
# all_cones = np.concatenate((all_cones, [starting_pos]))
# all_colors = np.concatenate((np.concatenate((color_yellow, color_blue)), color_orange))
# all_colors = np.concatenate((all_colors, color_green))

# plt.scatter(all_cones[:, 0], all_cones[:, 1], c=all_colors, s=5*4, cmap=plt.cm.Paired, marker='x')

# color_yellow = np.full(len(yellows), "#FFF300")
# color_blue = np.full(len(blues), "#3399FF")
# color_orange = np.full(len(oranges), "#FFA500")

# cones = np.concatenate((np.concatenate((yellows, blues)), oranges))
# colors = np.concatenate((np.concatenate((color_yellow, color_blue)), color_orange))

# # plt.scatter(cones[:, 0], cones[:, 1], c=colors, s=5*4, cmap=plt.cm.Paired, marker='o')

# plt.show()
