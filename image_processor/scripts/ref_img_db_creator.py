#!/usr/bin/env python

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
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped
import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from utils import *



############# module that takes images as inputs and output
# ######## intrusion deteection flag for UAV nodes ###
#master_wp = [ [32.5,-10.1], [28.6,-29], [12,-37.8], [-17.9,-37.8], [-35.68,-37.9], [-38.66,-15],\
#         [-38.6,14.88], [-34.4,32.44], [-15.48,38], [14.02,35.3], [30.75,30], [36,11.1] ]

master_wp = [[8.9, -4.88], [4.0, -7.045], [-2.254, -6.62], [-6.66, -3.815], [-7.77, 2.25], [-4.786, 6.96], [1.09, 8.48], [7.0, 7.0]]

img_diff_threshold = 1000


wpr0 = False
wpr1 = False

uav0_true_pose = PoseStamped()
uav1_true_pose = PoseStamped()


def uav0_local_pose_cb(data):
    global uav0_true_pose
    uav0_true_pose = data
    return uav0_true_pose

def uav1_local_pose_cb(data):
    global uav1_true_pose
    uav1_true_pose = data
    return uav1_true_pose


def if_waypoint_reached_0_cb(data):
	global wpr0
	wpr0 = (data.data)
    	return wpr0 

def if_waypoint_reached_1_cb(data):
	global wpr1
	wpr1 = (data.data)
    	return wpr1 



uav0_fpv = Image()
uav1_fpv = Image()
uav0_encoding = String()

def uav0_cam_img(data):
    global uav0_fpv
    uav0_fpv = data
    #uav0_encoding = data.encoding
    #print(uav0_fpv)
    return uav0_fpv

def uav1_cam_img(data):
    global uav1_fpv
    uav1_fpv = data
    return uav1_fpv

def myhook():
    print "shutting down.. all refernce images taken and saved"
    

def main():
    rospy.init_node('image_saver_server', anonymous=True)
    rate = rospy.Rate(5.0) 
    count = 0
    uav0_img_topic = "/iris_fpv_cam_0/camera_iris/image_raw"
    uav1_img_topic = "/iris_fpv_cam_1/camera_iris/image_raw"
    bridge = CvBridge()
    dir_path = '/home/ranjeet/catkin_ws/src/image_processor/image_database'
    
    while not rospy.is_shutdown():
        rospy.Subscriber("NWPR_0", Bool, if_waypoint_reached_0_cb, queue_size=1)
        rospy.Subscriber("NWPR_1", Bool, if_waypoint_reached_1_cb, queue_size=1)
        rospy.Subscriber("uav0/mavros/local_position/pose", PoseStamped, uav0_local_pose_cb, queue_size=1)
        rospy.Subscriber("uav1/mavros/local_position/pose", PoseStamped, uav1_local_pose_cb, queue_size=1)
        rospy.Subscriber(uav0_img_topic, Image, uav0_cam_img, queue_size=1)
        rospy.Subscriber(uav1_img_topic, Image, uav1_cam_img, queue_size=1)

        
	
        if wpr0 == True:
            #print(uav0_true_pose.pose.position.x, uav0_true_pose.pose.position.y, uav0_true_pose.pose.position.z)
	    #uav0_true_pose.x  = -7.6
	    #uav0_true_pose.y = 2.1
	    #print(uav0_true_pose.x, uav0_true_pose.y)
            uav_0_est_pos = est_curr_pose(uav0_true_pose.pose.position.x, uav0_true_pose.pose.position.y, master_wp)
	    
            #print('est_pose_0: ', uav_0_est_pos)
	    #print
            cwd = os.getcwd()
            directory = str(uav_0_est_pos[0]) + "_" + str(uav_0_est_pos[1])
            path = os.path.join(dir_path, directory)
	    #print('cwd: ', cwd)
	    #print('path: ', path)
	    
	    
            # save 5 images at this point
            [r, p, y] = quat_to_eul(uav0_true_pose.pose.orientation)
            print(uav_0_est_pos)
            if abs(r) <= 1.8 and abs(p)<=1.8:
                
                if os.path.isdir(path) == False:
                    print(os.path.isfile(path))
                    os.mkdir(path)
                    print('created a folder %s', path)
                else:
                     print('the folder %s afready exists', path)
                     
                for i in range(5):
                    filename = str(uav_0_est_pos[0]) + "_" + str(uav_0_est_pos[1])+ "--"+ str(i) + ".jpg"
                    filepath = os.path.join(path, filename)
                    #print(filepath)
                    if os.path.isfile(filepath) == False:
                        cv_img = bridge.imgmsg_to_cv2(uav0_fpv, desired_encoding='mono8') # convert to PIL image
                        cv2.imwrite(filepath, cv_img)
                        print('image: '+ str(i)+ ' saved at '+ str(uav_0_est_pos[0])+str(uav_0_est_pos[1]))
	    
            
            
        if wpr1 == True:
            uav_1_est_pos = est_curr_pose(uav1_true_pose.pose.position.x, uav1_true_pose.pose.position.y, master_wp)
            directory = str(uav_1_est_pos[0]) + "_" + str(uav_1_est_pos[1])
            path = os.path.join(dir_path, directory)
	    #print('est_pose_1: ', uav_1_est_pos)

            
            # save 5 images at this point
	    [r, p, y] = quat_to_eul(uav1_true_pose.pose.orientation)
            print(uav_1_est_pos)
            if abs(r) <= 1.8 and abs(p)<=1.8:
                
                if os.path.isdir(path) == False:
                    print(os.path.isfile(path))
                    os.mkdir(path)
                    print('created a folder %s', path)
                else:
                     print('the folder %s afready exists', path)
                
                
                for i in range(5):
                    filename = str(uav_1_est_pos[0]) + "_" + str(uav_1_est_pos[1])+ "--"+ str(i) + ".jpg"
                    filepath = os.path.join(path, filename)
                    #print(filepath)
                    if os.path.isfile(filepath) == False:
                        cv_img = bridge.imgmsg_to_cv2(uav1_fpv, desired_encoding='mono8') # convert to PIL image
                        cv2.imwrite(filepath, cv_img)
                        print('image: '+ str(i)+ ' saved at '+ str(uav_1_est_pos[0])+str(uav_1_est_pos[1]))

        
	 
        rate.sleep()
        
        if len(os.listdir(dir_path))== len(master_wp):
            rospy.signal_shutdown("all images saved at ref points")
            rospy.on_shutdown(myhook)


if __name__ == '__main__':

	try:
		main()
		
	except rospy.ROSInterruptException:
		pass
