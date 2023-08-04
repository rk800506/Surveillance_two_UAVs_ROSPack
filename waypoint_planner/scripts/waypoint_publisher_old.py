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

#mission_dict = dict{'intrusion':True, 'normal_surveillanse':False}

#master_wp = [[-115.5, 175.0], [-132.0,190.0], [-150.0,208.0],[-170.0,222.0],[-190.0,238.0]]
master_wp = [[-1.0,1.0], [0.0,2.0], [1.0,2.0],[2.0,1.0],[-1.0,0.0],[0.0,-1.0], [1.0,-1.0], [2.0,0.0]]


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
        if curr_set == strt_set:
                return rev_set
        else:
                return strt_set



pub0 = rospy.Publisher("uav0_nxt_wp", PoseStamped, queue_size=1)
pub1 = rospy.Publisher("uav1_nxt_wp", PoseStamped, queue_size=1)
################################  Main    #####################################**
def main():
        rospy.init_node('waypoint_publisher', anonymous=True)
	rate= rospy.Rate(1.0)
	Pose0 = PoseStamped()
        Pose1 = PoseStamped()

	uav_wp_st = distribute_wp(master_wp, uav_number)
        uav_wp_rev = reverse_waypoint_list(uav_wp_st)
        #print(uav_wp_st)
        #print(uav_wp_rev)

        # initiate waypoints
        wp_set_0 = uav_wp_st[0]
        wp_set_1 = uav_wp_st[1]
        index_0 = 0
        index_1 = 0
        while not rospy.is_shutdown():
                next_wp_0 = wp_set_0[index_0]
                next_wp_1 = wp_set_1[index_1]
                z = 3.0
                rospy.Subscriber("NWPR_0", Bool, if_waypoint_reached_0, queue_size=1)
                if wpr0 == True:
                        index_0 += 1
                        #print('index_0',index_0)
                        if index_0 == len(wp_set_0):
                                #print('im am inside wp togglising')
                                wp_set_0 = toggle_wp_set(wp_set_0, uav_wp_st[0], uav_wp_rev[0])
                                index_0 = 0
                        next_wp_0 = wp_set_0[index_0]
                        
                        

                rospy.Subscriber("NWPR_1", Bool, if_waypoint_reached_1, queue_size=1)
                if wpr1 == True:
                        index_1 += 1
                        if index_1 == len(wp_set_1):
                                wp_set_1 = toggle_wp_set(wp_set_1, uav_wp_st[1], uav_wp_rev[1])
                                index_1 = 0
                        next_wp_1 = wp_set_1[index_1]
        
                
                # publish waypoint to respective topics
                Pose0.pose.position.x = next_wp_0[0]
                Pose0.pose.position.y = next_wp_0[1]
                Pose0.pose.position.z = z

                Pose1.pose.position.x = next_wp_1[0]-1
                Pose1.pose.position.y = next_wp_1[1]
                Pose1.pose.position.z = z

                pub0.publish(Pose0)
                pub1.publish(Pose1)

                print('uav0: ',next_wp_0[0],next_wp_0[1], 'uav1: ', next_wp_1[0], next_wp_1[1])

                rate.sleep()
                
        
        

if __name__ == '__main__':

	try:
		main()
		
	except rospy.ROSInterruptException:
		pass



