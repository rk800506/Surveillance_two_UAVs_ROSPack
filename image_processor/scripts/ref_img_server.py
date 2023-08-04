# topics required
# waypoiny length topic
# topic that publishes camera ouput (uav/image_raw... name can be different)
# current waypoint  and next waypoint of all UAVs
# boolean topic that confirms that current waypoint has been reached (if reached then automatically set to '1')
#

# this program saves images at checkpoints as reference images at once at the mission start (reference image taking process
# for actual surveillance task)
# also this program acts as server for reference images when image differencing happens during surveillance task
# the program must have a directory alloted to keep reference images or to save any image.


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
from std_msgs.msg import Float64, Float32, Bool, Int16
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import os
import cv2
import numpy as np


############# module that takes images as inputs and output
# ######## intrusion deteection flag for UAV nodes ###
def master_wp_len_cb(data):
    master_wp_len = data.data
    return master_wp_len

# load reference image database
rospy.Subscriber("master_wp_len", Int16, master_wp_len_cb, queue_size=1)
#waypoint_list_len = 10  #   some no just given for experiemntation
img_database_path = 'img_director_path'
img_dir_list = os.listdir(img_database_path)
img_dir_len = len(img_dir_list)
#img_dir_len =10
####################assert master_wp_len == img_dir_len, "length of database img dir and waypoint list must be same!"

img_diff_threshold = 1000

img_in  = Image()###################### DOUBT ###########################

wpr0 = False

curr_pose PoseStamped() ############### doubt #################

def current_reached_wp(data):
    global curr_pose
    curr_pose = data.data
    x = curr_pose.pose.position.x
    y = curr_pose.pose.position.y
    z = curr_pose.pose.position.z
    coord = [x, y, z]
    return coord


def if_waypoint_reached_0(data):
	global wpr0
	wpr0 = (data.data)
    	return wpr0 
'''
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
'''
def camera_input(data):
    global img_in
    img_in = data.data
    return img_in

def main():
    rospy.init_node('image_saver_server', anonymous=True)
    rate = rospy.Rate(5.0) 

    while not rospy.is_shutdown():
        if wpr0 == True:
            
        
        rate.sleep()



if __name__ == '__main__':

	try:
		main()
		
	except rospy.ROSInterruptException:
		pass
