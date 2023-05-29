import numpy as np
import math
import matplotlib.pyplot as plt
import sys
import yaml

# theta  = (2 * pi * theta) / 6400 
# pose(x, y) = (d * cos(theta), d * sin(theta))

# Starting Postion
# start_pos = [[-15, 0]]
# starting_pos = np.zeros(2)
# starting_pos[0] = math.hypot(start_pos[0], -start_pos[1])
# bearing = math.atan2(-start_pos[1], start_pos[0])
# starting_pos[1] = (bearing * 6400) / (2 * math.pi)

# Cone Position ground truth
blues_real = [[ -0.0, 1.6500000000000004],
            [ -2.860558656929046, 2.2190004944781316],
            [ -5.285623189369442, 3.839376810630557],
            [ -6.905999505521868, 6.264441343070954],
            [ -7.475, 9.125],
            [ -6.905999505521868, 11.985558656929046],
            [ -5.285623189369443, 14.410623189369442],
            [ -2.860558656929047, 16.03099950552187],
            [ -9.154234823626465e-16, 16.6],
            [ 2.860558656929045, 16.03099950552187],
            [ 5.285623189369442, 14.410623189369444],
            [ 6.905999505521867, 11.98555865692905],
            [ 7.475, 9.125000000000002],
            [ 6.9059995055218675, 6.264441343070953],
            [ 5.285623189369444, 3.839376810630559],
            [ 2.8605586569290504, 2.2190004944781334],
            [ 7.619075567285049, -1.5059244327149495],
            [ 9.954801962809116, -5.001586016266157],
            [ 10.775, -9.125],
            [ 9.954801962809116, -13.248413983733842],
            [ 7.6190755672850505, -16.744075567285048],
            [ 4.123413983733844, -19.079801962809114],
            [ 1.3195569260812732e-15, -19.9],
            [ -4.123413983733841, -19.079801962809114],
            [ -7.619075567285049, -16.74407556728505],
            [ -9.954801962809112, -13.248413983733847],
            [ -10.775, -9.125000000000002],
            [ -9.954801962809114, -5.001586016266155],
            [ -7.619075567285051, -1.5059244327149521]]

yellows_real = [[ -7.619075567285049, 1.5059244327149495],
            [ -9.954801962809116, 5.001586016266157],
            [ -10.775, 9.125],
            [ -9.954801962809116, 13.248413983733842],
            [ -7.6190755672850505, 16.744075567285048],
            [ -4.123413983733844, 19.079801962809114],
            [ -1.3195569260812732e-15, 19.9],
            [ 4.123413983733841, 19.079801962809114],
            [ 7.619075567285049, 16.74407556728505],
            [ 9.954801962809112, 13.248413983733847],
            [ 10.775, 9.125000000000002],
            [ 9.954801962809114, 5.001586016266155],
            [ 7.619075567285051, 1.5059244327149521],
            [ 0.0, -1.6500000000000004],
            [ 2.860558656929046, -2.2190004944781316],
            [ 5.285623189369442, -3.839376810630557],
            [ 6.905999505521868, -6.264441343070954],
            [ 7.475, -9.125],
            [ 6.905999505521868, -11.985558656929046],
            [ 5.285623189369443, -14.410623189369442],
            [ 2.860558656929047, -16.03099950552187],
            [ 9.154234823626465e-16, -16.6],
            [ -2.860558656929045, -16.03099950552187],
            [ -5.285623189369442, -14.410623189369444],
            [ -6.905999505521867, -11.98555865692905],
            [ -7.475, -9.125000000000002],
            [ -6.9059995055218675, -6.264441343070953],
            [ -5.285623189369444, -3.839376810630559],
            [ -2.8605586569290504, -2.2190004944781334]]

oranges_real = [[ -11.25, 1.65],
            [ -11.25, -1.65],
            [ -15.0, 1.65],
            [ -15.0, -1.65],
            [ 11.0, 1.65],
            [ 11.0, -1.65],
            [ 16.0, 1.65],
            [ 16.0, -1.65],
            [ 21.0, 1.65],
            [ 21.0, -1.65],
            [ 21.0, 0.0],
            [ 21.0, 1.65]]

big_orange_real = [[ -1.3, 2.0],
                [ -1.3, -2.0],
                [ 1.3, 2.0],
                [ 1.3, -2.0]]
                    

# Convert from (x, y) to (d, theta)
blue_cones = []
for blue in blues_real:
    distance = math.hypot(blue[0], -blue[1])
    bearing = math.atan2(-blue[1], blue[0])
    theta = (bearing * 6400) / (2 * math.pi)
    if theta < 0 :
        theta += 6400
    val = []
    val.append(distance)
    val.append(theta)
    blue_cones.append([val[0], val[1]])

yellow_cones = []
for yellow in yellows_real:
    distance = math.hypot(yellow[0], -yellow[1])
    bearing = math.atan2(-yellow[1], yellow[0])
    theta = (bearing * 6400) / (2 * math.pi)
    if theta < 0 :
        theta += 6400
    val = []
    val.append(distance)
    val.append(theta)
    yellow_cones.append([val[0], val[1]])

orange_cones = []
for orange in oranges_real:
    distance = math.hypot(orange[0], -orange[1])
    bearing = math.atan2(-orange[1], orange[0])
    theta = (bearing * 6400) / (2 * math.pi)
    if theta < 0 :
        theta += 6400
    val = []
    val.append(distance)
    val.append(theta)
    orange_cones.append([val[0], val[1]])

big_orange_cones = []
for orange_big in big_orange_real:
    distance = math.hypot(orange_big[0], -orange_big[1])
    bearing = math.atan2(-orange_big[1], orange_big[0])
    theta = (bearing * 6400) / (2 * math.pi)
    if theta < 0 :
        theta += 6400
    val = []
    val.append(distance)
    val.append(theta)
    big_orange_cones.append([val[0], val[1]])

# Save to .yaml file
file = open('skidpadVN.yaml', 'w')
file.write("blue_cones: \n")
yaml.dump(blue_cones, file)
file.write("yellow_cones: \n")
yaml.dump(yellow_cones, file)
file.write("orange_cones: \n")
yaml.dump(orange_cones, file)
file.write("big_orange_cones: \n")
yaml.dump(big_orange_cones, file)
file.close()


# Visualization stuff
color_yellow = np.full(len(yellow_cones), "#FFF300")
color_blue = np.full(len(blue_cones), "#3399FF")
color_orange = np.full(len(orange_cones), "#FFA500")
color_big_orange = np.full(len(big_orange_cones), "#FF5E13")
color_green =  np.full(1, "#17A536")

all_cones = np.concatenate((np.concatenate((yellow_cones, blue_cones)), orange_cones))
all_cones = np.concatenate((all_cones, big_orange_cones))

all_colors = np.concatenate((np.concatenate((color_yellow, color_blue)), color_orange))
all_colors = np.concatenate((all_colors, color_big_orange))

plt.scatter(all_cones[:, 0], all_cones[:, 1], c=all_colors, s=5*4, cmap=plt.cm.Paired, marker='x')
plt.show()