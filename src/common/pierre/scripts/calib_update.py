#!/usr/bin/env python3

import oyaml as yaml
import math
import numpy as np
import os
import argparse
import rospkg

def read_yaml(path_file):
    with open(path_file, 'r') as stream:
        try:
            file = yaml.load(stream, Loader= yaml.Loader)
        except yaml.YAMLError as exc:
            print(exc)
    return file


def write_yaml (path_file, data):
    with open(path_file, 'w') as file:
        yaml.dump(data, file, default_flow_style=None)


def create_matrix(path_pkg, step_camera):
    path_file = path_pkg + "/config/sensors/dump_file.yaml"
    file = read_yaml(path_file)

    print("\nCamera %i parameters: " %step_camera)
    
    if step_camera == 1:
        rx = file["left_rx"]
        print("rx: ",rx)
        ry = file["left_ry"]
        print("ry: ",ry)
        rz = file["left_rz"]
        print("rz: ",rz)
        tx = file["left_tx"]
        print("tx: ",tx)
        ty = file["left_ty"]
        print("ty: ",ty)
        tz = file["left_tz"]
        print("tz: ",tz)
    
    else:
        rx = file["right_rx"]
        print("rx: ",rx)
        ry = file["right_ry"]
        print("ry: ",ry)
        rz = file["right_rz"]
        print("rz: ",rz)
        tx = file["right_tx"]
        print("tx: ",tx)
        ty = file["right_ty"]
        print("ty: ",ty)
        tz = file["right_tz"]
        print("tz: ",tz)

    rx = math.radians(rx)
    ry = math.radians(ry)
    rz = math.radians(rz)

    rx_mat = np.array([[1,      0,              0],
                    [0,      math.cos(rx),   -math.sin(rx)],
                    [0,      math.sin(rx),   math.cos(rx)]])

    ry_mat = np.array([[math.cos(ry),   0,    math.sin(ry)],
                    [0,              1,    0],
                    [-math.sin(ry),  0,    math.cos(ry)]])

    rz_mat = np.array([[math.cos(rz),  -math.sin(rz),    0],
                    [math.sin(rz),   math.cos(rz),    0],
                    [0,              0,               1]])

    rot_mat = np.dot(np.dot(rx_mat,ry_mat),rz_mat)

    print("Rotation Matrix: \n",rot_mat)

    trans_mat = np.array([[tx, ty, tz]])
    
    print("Translation Matrix: \n",trans_mat)

    return rot_mat, trans_mat

def update_calib (path_pkg, cfg_file, out):
    
    path_calib = path_pkg + "/config/sensors/" + cfg_file
    calib_file = read_yaml(path_calib)

    n_cameras = 1
    if "camera_number" in calib_file["common"]:
        n_cameras = calib_file["common"]["camera_number"]

    if out == "matrix":

        print(r"""

                 ______      _             ___    __  __       _        _      
                |  ____|    | |           |__ \  |  \/  |     | |      (_)     
                | |__  _   _| | ___ _ __     ) | | \  / | __ _| |_ _ __ ___  __
                |  __|| | | | |/ _ \ '__|   / /  | |\/| |/ _` | __| '__| \ \/ /
                | |___| |_| | |  __/ |     / /_  | |  | | (_| | |_| |  | |>  < 
                |______\__,_|_|\___|_|    |____| |_|  |_|\__,_|\__|_|  |_/_/\_\                                                           

                """)

        # Dump actual dynamic reconfigure parameters
        path_dump = path_pkg + "/config/sensors/dump_file.yaml"
        os.system("rosrun dynamic_reconfigure dynparam dump /sensor_fusion " + path_dump)

        if n_cameras == 1 :
            rot_mat, trans_mat = create_matrix(path_pkg,n_cameras)

            # Subsitute yaml file with new rot and trans matrices
            calib_file["common"]["R_left"] = rot_mat.flatten().tolist()
            calib_file["common"]["T_left"] = trans_mat.flatten().tolist()

        else: 
            step_camera = 1
            rot_mat1, trans_mat1 = create_matrix(path_pkg,step_camera)

            # Subsitute yaml file with new rot and trans matrices
            calib_file["common"]["R_left"] = rot_mat1.flatten().tolist()
            calib_file["common"]["T_left"] = trans_mat1.flatten().tolist()

            step_camera = 2
            
            rot_mat2, trans_mat2 = create_matrix(path_pkg, step_camera)

            # Subsitute yaml file with new rot and trans matrices
            calib_file["common"]["R_right"] = rot_mat2.flatten().tolist()
            calib_file["common"]["T_right"] = trans_mat2.flatten().tolist()


        write_yaml(path_calib,calib_file)

        # Remove temp reconfig file
        os.remove(path_dump)
    
    if out == "euler":

        print(r"""
                 __  __       _        _        ___    ______      _           
                |  \/  |     | |      (_)      |__ \  |  ____|    | |          
                | \  / | __ _| |_ _ __ ___  __    ) | | |__  _   _| | ___ _ __ 
                | |\/| |/ _` | __| '__| \ \/ /   / /  |  __|| | | | |/ _ \ '__|
                | |  | | (_| | |_| |  | |>  <   / /_  | |___| |_| | |  __/ |   
                |_|  |_|\__,_|\__|_|  |_/_/\_\ |____| |______\__,_|_|\___|_|   
                                                                
                """)
        
        rot_mat1 = calib_file["common"]["R_left"]
        trans_mat1 = calib_file["common"]["T_left"]
        print("Euler Angles Left: ")
        x = math.atan2(-rot_mat1[5] , rot_mat1[8])
        print("X: ", math.degrees(x))
        y = math.asin(rot_mat1[2])
        print("Y: ", math.degrees(y))
        z = math.atan2(-rot_mat1[1], rot_mat1[0])
        print("Z: ",math.degrees(z))

        print("Translation Left: ")
        tx = trans_mat1[0]
        print("tx: ", tx)
        ty = trans_mat1[1]
        print("ty: ", ty)
        tz = trans_mat1[2]
        print("tz: ", tz)

        if n_cameras == 2:
            rot_mat2 = calib_file["common"]["R_right"]
            trans_mat2 = calib_file["common"]["T_right"]
            print("Euler Angles Right: ")
            x = math.atan2(-rot_mat2[5] , rot_mat2[8])
            print("X: ", math.degrees(x))
            y = math.asin(rot_mat2[2])
            print("Y: ", math.degrees(y))
            z = math.atan2(-rot_mat2[1], rot_mat2[0])
            print("Z: ", math.degrees(z))

            print("Translation Right: ")
            tx = trans_mat2[0]
            print("tx: ", tx)
            ty = trans_mat2[1]
            print("ty: ", ty)
            tz = trans_mat2[2]
            print("tz: ", tz)


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--cfg_file', required = True, type = str, help ='Config file to write new parameters')
    parser.add_argument('--out', default = "matrix" , type = str, help ='Config file to write new parameters')
    args = parser.parse_args()
    
    path_pkg = rospack.get_path('common_meta_pkg')
    
    update_calib (path_pkg, args.cfg_file, args.out)