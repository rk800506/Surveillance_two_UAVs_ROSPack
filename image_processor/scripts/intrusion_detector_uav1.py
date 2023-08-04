#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import roslib
import math
import time
import tf
from math import *
from pylab import genfromtxt; 
from std_msgs.msg import String
from std_msgs.msg import Float64, Float32, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import os
import cv2
import numpy as np


############# module that takes images as inputs and output
# ######## intrusion deteection flag for UAV nodes ###

# load reference image database
waypoint_list_len = 10  #   some no just given for experiemntation
img_database_path = 'img_director_path'
img_dir_list = os.listdir(img_database_path)
img_dir_len = len(img_dir_list)
img_dir_len =10
assert waypoint_list_len == img_dir_len, "length of database img dir and waypoint list must be same!"

img_diff_threshold = 1000

img_in  = Image()###################### DOUBT ###########################

wpr1 =False
curr_pose PoseStamped()

def current_reached_wp(data):
    global curr_pose
    curr_pose = data.data
    x = curr_pose.pose.position.x
    y = curr_pose.pose.position.y
    z = curr_pose.pose.position.z
    coord = [x, y, z]
    return coord

def if_waypoint_reached_1(data):
	global wpr1
	wpr1 = (data.data)
    	return wpr1

def select_img_from_database(uav_curr_waypoint_reached):
    x1,y1,z1 = uav_curr_waypoint_reached
    ref_img_file_name  = str(x1)+'_'+str(y1)+'_'+str(z1)+'_img_'
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

def camera_input(data):
    global img_in
    img_in = data.data
    return img_in

def main():
    rospy.init_node('intrusion_detector', anonymous=True)
    rate = rospy.Rate(2.0) 
    intrusion_detected = Bool()
    pub = rospy.Publisher('intrusion_uav1', Bool, queue_size=1)
    count = 0
    while not rospy.is_shutdown():
        rospy.init_node('intrusion_detector_uav1', anonymous=True)
        ### image sequence input ####
        #count += 1
        rospy.Subscriber("uav1/camera_input_raw", Image, camera_input, queue_size=1)
        rospy.Subscriber("NWPR_1", Bool, if_waypoint_reached_1, queue_size=1)
        rospy.Subscriber("uav1/current_wp_reached", PoseStamped, current_reached_wp, )


        ##### Processing #######
        if NWR0 == True:
            diff = []
            ref_img_from_db =  select_img_from_database(coord)
            for ref_img in ref_img_from_db:
                diff.append(sum(abs(ref_img - img_in)))
            if min(diff) > img_diff_threshold:
                intrusion_detected = True
            else:
                intrusion_detected = False
                
        ##########

        
        #img_diff = 999
        
        #if count > 60 and count < 150:
        #    img_diff = 1001
        

        #f img_diff >= img_diff_threshold:
        #    intrusion_detected = True
        #else:
        #    intrusion_detected = False
        
        ###########

        ########## publish appropriate flag ###########3
        pub.publish(intrusion_detected)

        rate.sleep()



if __name__ == '__main__':

	try:
		main()
		
	except rospy.ROSInterruptException:
		pass
