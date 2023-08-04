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
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import *
import random
from sensor_msgs.msg import BatteryState
from multiprocessing import Process

wpr0 = False
wpr1 = False
uav_number = 2
intrusion_flag_uav0 = False
intrusion_flag_uav1 = False

#master_wp = [[-115.5, 175.0], [-132.0,190.0], [-150.0,208.0],[-170.0,222.0],[-190.0,238.0]]
#master_wp = [[-1.0,1.0], [0.0,2.0], [1.0,2.0],[2.0,1.0],[-1.0,0.0],[0.0,-1.0], [1.0,-1.0], [2.0,0.0]]
#master_wp = [[56.59,-1.36], [65.45,-19.13], [58.42,-46], [35,-64.58], [16.22,-64.9], \
#        [-13.3,-74.5], [-42.89,-79], [-71.68,-76], [-93.31,-57.5], [-97.51,-28.34], \
#        [-99.51,1.4], [-101.55,31.22], [-100.26,50.57], [-74.47,63.84], [-46.34,61.9], \
#        [-17,57.6], [12.24,53.25], [38.57,41.63], [52.9,33.65], [51.65,13.7]]

#master_wp = [ [32.5,-10.1], [28.6,-29], [12,-37.8], [-17.9,-37.8], [-35.68,-37.9], [-38.66,-15],\
#         [-38.6,14.88], [-34.4,32.44], [-15.48,38], [14.02,35.3], [30.75,30], [36,11.1] ]

master_wp = [[8.9, -4.88], [4.0, -7.045], [-2.254, -6.62], [-6.66, -3.815], [-7.77, 2.25], [-4.786, 6.96], [1.09, 8.48], [7.0, 7.0]]

def intrusion_uav0_cb(data):
        global intrusion_flag_uav0
        intrusion_flag_uav0 = data.data
        return intrusion_flag_uav0

def intrusion_uav1_cb(data):
        global intrusion_flag_uav1
        intrusion_flag_uav1 = data.data
        return intrusion_flag_uav1

def if_waypoint_reached_0(data):
	global wpr0
	wpr0 = (data.data)
    	return wpr0 

def if_waypoint_reached_1(data):
	global wpr1
	wpr1 = (data.data)
    	return wpr1

def distribute_wp(master_wp, uav_number):
        uav_wp = [[]]*uav_number
        #print(uav_wp)
        extra_wp = int(len(master_wp)%uav_number)
        
        wp_len = int(len(master_wp)/uav_number)
        #print('extra_wp', extra_wp, 'wp_len', wp_len)
        #print(master_wp[0:3])
        for i in range(uav_number):
                if i+1 <= extra_wp:
                        uav_wp[i] = master_wp[i*(wp_len+1):(i+1)*(wp_len+1)]
                        #print 'uav_', i,uav_wp[i]
                else:
                        uav_wp[i] = master_wp[i*wp_len+extra_wp:(i+1)*wp_len+extra_wp]
                        #print 'uav_',i,uav_wp[i]
        return uav_wp

def reverse_list(st_list):
        rev_list = [[]]*len(st_list)
        #print(len(st_list))
        for i in range(len(st_list)):
                rev_list[len(st_list)-1-i] = st_list[i]    
        return rev_list

        
def reverse_waypoint_list(uav_wp_strt):
        a = len(uav_wp_strt)
        uav_wp_rev = []
        for i in range(a):
                uav_wp_rev.append(reverse_list(uav_wp_strt[i]))
        return uav_wp_rev

def toggle_wp_set(curr_set, strt_set, rev_set):
        #print(len(strt_set), len(rev_set))
        if curr_set == strt_set:
                return rev_set
        else:
                return strt_set



pub0 = rospy.Publisher("uav0_nxt_wp", PoseStamped, queue_size=1)
pub1 = rospy.Publisher("uav1_nxt_wp", PoseStamped, queue_size=1)
pub2 = rospy.Publisher("master_wp_len", Int16, queue_size=1)
################################  Main    #####################################**
def main():
    rospy.init_node('waypoint_publisher', anonymous=True)
    rate = rospy.Rate(1.0)
    Pose0 = PoseStamped()
    Pose1 = PoseStamped()
    master_wp_len  =Int16()

    uav_wp_st = distribute_wp(master_wp, uav_number)
    uav_wp_rev = reverse_waypoint_list(uav_wp_st)
    
    #print(uav_wp_st)
    #print(uav_wp_rev)

    # initiate waypoints
    wp_set_0 = uav_wp_st[0]
    wp_set_1 = uav_wp_st[1]
    index_0 = 0
    index_1 = 0
    h_off = 1
    do_not_set_wp_twice_0 = False
    do_not_set_wp_twice_1 = False

    next_wp_0 = wp_set_0[index_0]
    next_wp_1 = wp_set_1[index_1]
        
    while not rospy.is_shutdown():
        
        
        z1 = 5.0
        z0 = 5.0
        wp_master_cp = master_wp
        rospy.Subscriber("NWPR_0", Bool, if_waypoint_reached_0, queue_size=1)
        rospy.Subscriber('intrusion_uav0', Bool, intrusion_uav0_cb, queue_size=1)
        #print('uav0_intrusion: ', intrusion_flag_uav0)

        if  wpr0 == True and intrusion_flag_uav0 == True:
            intrusion_location = next_wp_0
            if intrusion_location in wp_master_cp:
                wp_master_cp.remove(intrusion_location)
            wp_set_1 = wp_master_cp
            #print('bro i m here: ', wp_set_1)
            z1 = z1+h_off
            next_wp_0 = intrusion_location
            #print('intrusion detetcted by uav0 at : ', intrusion_location)

        if wpr0 == True and intrusion_flag_uav0 == False and do_not_set_wp_twice_0 == False:
            index_0 += 1
            #print('index_0',index_0)
            if index_0 == len(wp_set_0):
                #print('im am inside wp togglising with index: ', index_0)
                #print('wp_prev', wp_set_0)
                wp_set_0 = toggle_wp_set(wp_set_0, uav_wp_st[0], uav_wp_rev[0])
                #print('wp_curr', wp_set_0)
                index_0 = 0
            
            next_wp_0 = wp_set_0[index_0]
            
            print('index_0',index_0, 'wpr0: ', wpr0, 'next_wp_0: ', next_wp_0)
            do_not_set_wp_twice_0 = True
                                                        
        else:
                #print('wpr0: ', wpr0)
                do_not_set_wp_twice_0 = False

        rospy.Subscriber("NWPR_1", Bool, if_waypoint_reached_1, queue_size=1)
        rospy.Subscriber('intrusion_uav1', Bool, intrusion_uav1_cb, queue_size=1)
        #print('uav1_intrusion: ', intrusion_flag_uav1)

        if  wpr1 == True and intrusion_flag_uav1==True:
            intrusion_location = next_wp_1
            if intrusion_location in wp_master_cp:
                wp_master_cp.remove(intrusion_location)
            wp_set_0 = wp_master_cp
            next_wp_1 = intrusion_location
            z0 = z0+h_off
            #print('intrusion detetcted by uav1 at: ', intrusion_location)
            

        if wpr1 == True and intrusion_flag_uav1 == False and do_not_set_wp_twice_1 ==  False:
            index_1 += 1
            #print('index_1', index_1)
            if index_1 == len(wp_set_1):
                wp_set_1 = toggle_wp_set(wp_set_1, uav_wp_st[1], uav_wp_rev[1])
                index_1 = 0
            next_wp_1 = wp_set_1[index_1]
            print('index_1', index_1, 'wpr1: ', wpr1, 'nect_wp_1: ',next_wp_1)
            do_not_set_wp_twice_1 = True
        else:
                do_not_set_wp_twice_1 = False
                #print('wpr1: ', wpr1)
                
        '''if intrusion_flag_uav0 == False and intrusion_flag_uav1 == False:
                #wp_set_0 = uav_wp_st[0]
                #wp_set_1 = uav_wp_st[1]
                if index_0 >= len(wp_set_0):
                        index_0 = 0
                if index_1 >= len(wp_set_1):
                        index_1 = 0
        '''
                
        #print('uav0-',  next_wp_1, wp_set_1)	#########################################################
        #print( 'uav1-',  next_wp_0, wp_set_0)
        print
        # publish waypoint to respective topics
        Pose0.pose.position.x = next_wp_0[0]-1
        Pose0.pose.position.y = next_wp_0[1]
        Pose0.pose.position.z = z0

        Pose1.pose.position.x = next_wp_1[0]  # -1 s due to bug of reference frames from world and from uav
        Pose1.pose.position.y = next_wp_1[1]
        Pose1.pose.position.z = z1

        #print('uav0_next_wp:', Pose0.pose.position.x+1, Pose0.pose.position.y)
        #print('uav1_nect_wp:', Pose1.pose.position.x, Pose1.pose.position.y)
        
        pub0.publish(Pose0)	##########################################################
        pub1.publish(Pose1)
        
        master_wp_len = len(master_wp)
        pub2.publish(master_wp_len)

        #print('uav0: ',next_wp_0[0],next_wp_0[1], 'uav1: ', next_wp_1[0], next_wp_1[1])
        

        rate.sleep()
                
        
        

if __name__ == '__main__':

	try:
		main()
		
	except rospy.ROSInterruptException:
		pass



