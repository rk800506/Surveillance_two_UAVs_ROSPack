#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import roslib
import math
import time
import tf
from math import *
import os
import cv2
import numpy as np




def est_curr_pose(x_exact, y_exact, master_wp):
    dist_prev = 10000
    for item in master_wp:
        dist = math.sqrt((item[0]-x_exact)**2 + (item[1]-y_exact)**2)
	#print('dist: ', dist)
        if dist < dist_prev:
            temp = item
            dist_prev = dist
	    #print(item, dist)
    return temp


def quat_to_eul(q):
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)*180/pi

    pitch = math.asin(2*(q.w*q.y - q.z*q.x))*180/pi

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

    yaw = math.atan2(siny_cosp, cosy_cosp)*180/pi

    return [roll, pitch, yaw]


def select_img_from_database(uav_0_est_pos):
    img_dir = str(uav_0_est_pos[0]) + "_" + str(uav_0_est_pos[1])
    ref_img_dir = os.path.join(img_database_path, img_dir)
    ref_images = []
    extension  = '.jpg'
    for i in range(2):  # for 2 images at any waypont location
        file_name = ref_img_file_name+str(i)+extension #   file extension depends upon the database
        for j in img_dir_list:
            if file_name == j:
                img = cv2.imread(img_database_path+'/'+j)
                img_np = np.array(img)
                ref_images.append(img_np)
    return ref_images
