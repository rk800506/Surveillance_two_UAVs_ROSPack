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
from cv_bridge import CvBridge
from utils import *


############# module that takes images as inputs and output
# ######## intrusion deteection flag for UAV nodes ###

# load reference image database
master_wp = [[8.9, -4.88], [4.0, -7.045], [-2.254, -6.62], [-6.66, -3.815], [-7.77, 2.25],\
             [-4.786, 6.96], [1.09, 8.48], [7.0, 7.0]]
img_database_path = '/home/ranjeet/catkin_ws/src/image_processor/image_database'
assert len(master_wp) == len(os.listdir(img_database_path)), "length of database img dir and waypoint list must be same!"

img_diff_threshold = 1000
wpr0 = False


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



uav0_fpv = Image()
def uav0_cam_img(data):
    global uav0_fpv
    uav0_fpv = data
    return uav0_fpv


uav0_true_pose = PoseStamped()
def uav0_local_pose_cb(data):
    global uav0_true_pose
    uav0_true_pose = data
    return uav0_true_pose


def main():
    rospy.init_node('intrusion_detector_uav0', anonymous=True)
    rate = rospy.Rate(2.0) 
    intrusion_detected = Bool()
    pub = rospy.Publisher('intrusion_uav0', Bool, queue_size=1)
    count = 0
    bridge = CvBridge()
    
    while not rospy.is_shutdown():
        ### image sequence input ####
        #count += 1
        rospy.Subscriber("/iris_fpv_cam_0/camera_iris/image_raw", Image, uav0_cam_img, queue_size=1)
        rospy.Subscriber("NWPR_0", Bool, if_waypoint_reached_0, queue_size=1)
        rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, uav0_local_pose_cb, queue_size=1)
        #rospy.Subscriber("uav0/current_wp_reached", PoseStamped, current_reached_wp, )


        ##### Processing #######
        print(wpr0)
        test_im_dir = '/home/ranjeet/catkin_ws/src/image_processor/test_img'
        
        if wpr0 == True:
            diff = []
            uav_0_est_pos = est_curr_pose(uav0_true_pose.pose.position.x, uav0_true_pose.pose.position.y, master_wp)
            
            img_dir = str(uav_0_est_pos[0]) + "_" + str(uav_0_est_pos[1])
            ref_img_dir = os.path.join(img_database_path, img_dir)
            
            [r, p, y] = quat_to_eul(uav0_true_pose.pose.orientation)
            
            if abs(r) <= 1.8 and abs(p)<=1.8:
                for ref_img in os.listdir(ref_img_dir):

                    cv_img_ref = cv2.imread(os.path.join(ref_img_dir, ref_img), 0)
                    #img_msg = bridge.cv2_to_imgmsg(cv_img_ref, "passthrough")
                    #cv2.imshow( 'ref_img', cv_img_ref)
                    np_img = np.asarray(cv_img_ref)
                    #print(np.max(np_img), np.min(np_img))
                    print(np_img.shape)
                    #print(uav0_fpv.encoding)
                    fpv_img_cv = bridge.imgmsg_to_cv2(uav0_fpv, "mono8")

                    
                    #test_img_directory = os.path.join(test_im_dir, ref_img)
                    #print(test_img_directory)
                    #cv2.imwrite(test_img_directory, fpv_img_cv)
                    
                    
                    #print(fpv_img_cv.shape)
                    #fpv_img_cv = cv2.cvtColor(fpv_img_cv, cv2.COLOR_RGB2GRAY)
                    #cv2.imshow('fpv_img', fpv_img_cv)
                    
                    fpv_img_np_arr = np.asarray(fpv_img_cv)
                    
                    #print(fpv_img_np_arr.shape)
                    #print(uav0_fpv)
                    diff.append(np.sum(abs(fpv_img_np_arr - np_img)))
                    #print('diff_max: ',np.max(abs(fpv_img_np_arr - np_img)), np.min(abs(fpv_img_np_arr - np_img)))
                    result = np.where(abs(fpv_img_np_arr - np_img) == 150)
                    print(len(result[0]))
                    #cv2.waitKey(0)  
                    #closing all open windows  
                    #cv2.destroyAllWindows()
                

                print(diff)
                if max(diff) > img_diff_threshold:
                    intrusion_detected = False
                    print('intrusion detected at - ', uav_0_est_pos)
                else:
                    intrusion_detected = False
                    print('no intrusion at - ', uav_0_est_pos)
                    
       
        pub.publish(intrusion_detected)

        rate.sleep()



if __name__ == '__main__':

	try:
		main()
		
	except rospy.ROSInterruptException:
		pass
