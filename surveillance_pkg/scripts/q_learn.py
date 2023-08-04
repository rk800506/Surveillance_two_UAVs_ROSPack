#!/usr/bin/env python

# read model state of model:'name_qc1_2', body:'link' using GetModelState
# apply body wrench in W frame for thrust

import numpy as np
#import skfuzzy as fuzz
import matplotlib.pyplot as plt
import rospy
import roslib
import math
import time
import tf
from math import *
from pylab import genfromtxt; 
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from sensor_msgs.msg import BatteryState

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import *
import random

MAXSTATES = 4**2
N = 10 # number of episode
GAMMA = 0.9
ALPHA = 0.01

bins = np.zeros((2,4))
bins[0] = np.linspace(0, 102, 4)



print(np.linspace(0, 102, 4))
bins[1] = np.linspace(0, 102, 4)

v1 = 100.0
v2 = 100.0
action_taken1 = 100
action_taken2 = 100
count_move_to_base = 0

Q = {}
all_states = []
for i in range(4):
	for j in range(4):
		all_states.append(str(i*10+j).zfill(2))
print('all_states', all_states)
for state in all_states:
	Q[state] = {}
	for action in range(3):
		Q[state][action] = 0
Q1 =  {'02': {0: 0, 1: 0, 2: 0}, '03': {0: -59.7, 1: -0.0996418, 2: 0}, '00': {0: 0, 1: 0, 2: 0}, '01': {0: 0, 1: 0, 2: 0}, '20': {0: 0, 1: 0, 2: 0}, '21': {0: -0.1542385913749425, 1: -59.69946176577446, 2: -0.05480981746360033}, '22': {0: 0.059768283816191904, 1: -0.09927314529419257, 2: 0}, '10': {0: 0, 1: 0, 2: 0}, '33': {0: 0.6340541221565176, 1: -0.7433965591831994, 2: -1.3689179367607578}, '32': {0: 0.21818867544428652, 1: -0.2959363204163864, 2: -0.2948234372471904}, '31': {0: 0.008352106490665662, 1: -118.21089491679163, 2: 0.13808071094387647}, '23': {0: 0.4109033692329494, 1: -0.19831108248170337, 2: -0.29730299668403265}, '13': {0: -30.07893218261417, 1: -0.2970678346624611, 2: -29.87626087411064}, '12': {0: 0.10120378658014678, 1: -0.19845895205800002, 2: 0}, '30': {0: 0, 1: 0, 2: 0}, '11': {0: -0.020321456324815537, 1: -231.76325818511904, 2: 0.08340153982412776}}
print('initial Q value', Q)

observation = np.zeros(2)
pub_1 = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
pub_2 = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)

def callback1(data):
	global v1
	#print('data.data from callback 1: ', data.voltage)
	v1 = data.voltage
	#print('in cll back: ', type(v1))
    	return v1 

def callback2(data):
	global v2
	v2 = data.data
    	return v2

def callback3(data):
	global action_taken1
	action_taken1 = data.pose
	print(action_taken1)
    	return action_taken1

def callback4(data):
	global action_taken2
	action_taken2 = data.pose
    	return action_taken2

def plot_running_avg(total_rewards):
	N = len(total_rewards)
	running_avg = np.empty(N)
	for t in range(N):
		running_avg[t] = total_rewards[max(0, t)]#running_avg[t] = np.mean(total_rewards[max(0, t-100):(t+1)])
	plt.plot(running_avg)
	plt.title("Running Avarage")
	plt.show()

def max_dict(d):
	max_v = float('-inf')
	for key, val in d.items():
		if val > max_v:
			max_v = val
			max_key = key
	return max_key, max_v

def assign_bins(observation, bins):
	state = np.zeros(2)
	for i in range(2):
		state[i] = np.digitize(observation[i], bins[i])
	return state

def get_state_as_string(state):
	string_state = ''.join(str(int(e)) for e in state)
	return string_state


def play_many_games(N):
	global Q
	global all_states
	global v1
	global v2
	global action_taken1
	global action_taken2
	global count_move_to_base
	global t0

	rospy_Rate= 4
	rate = rospy.Rate(rospy_Rate) 
	T=0.1
		
	act =  0
	observation = np.zeros(2)
	lengthm = []
	rewardm = []
	for n in range(N):
		now1 = rospy.get_rostime()
		t1 =  now1.secs%1000+round(now1.nsecs/10**6,0)*0.001
		#print('n',n)
		#eps = 0.5/(1+n*10e-3)
		eps = 1/np.sqrt(n+1)
		#eps=0.5
		i=0
		episode_length=0

		#done1 = True
		#done2 = True
		#done = False

 		rospy.Subscriber("/uav0/mavros/battery", BatteryState, callback1, queue_size=1)
		#print('v1: ........... ',v1)
		observation[0] = v1
		#print()
		#print('...................................................', v1)
		rospy.Subscriber("/uav1/mavros/battery", BatteryState, callback2, queue_size=1)
		observation[1] = v2
		state = get_state_as_string(assign_bins(observation, bins))
		#print('observation',observation)

		#print('state',state)
		episode_reward = 0

		while not rospy.is_shutdown():
			i+=1
			rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, callback3, queue_size=1)
			#rospy.Subscriber("/uav0/mavros/battery", BatteryState, callback1, queue_size=1)
			#print(v1)
			rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, callback4, queue_size=1)
			print('action_taken1,2_topic',action_taken1, action_taken2)
			
			#....take some decision using Q-learning........#
			Pose = PoseStamped() 
			Pose.pose.position.x = 3
			Pose.pose.position.x = 4
			Pose.pose.position.x = 8
			pub_1.publish(Pose)

			Pose.pose.position.x = -3
			Pose.pose.position.x = 4
			Pose.pose.position.x = 8
			pub_2.publish(Pose)

			rate.sleep()

		'''print('Qf1---- Loop ended, iteration:',n, 'episode_length:',episode_length,'episode_reward:',episode_reward)
		print('Qf2---- Final Q value:', Q)
		outFile.write("""Q values start, episode = %d\n""" % n)
		outFile.write("""episode_length = %d\n""" % episode_length)
		outFile.write("""episode_reward = %d\n""" % episode_reward)
		outFile.write("""total move to base = %d\n""" % count_move_to_base)
		outFile.write(str(Q))
		outFile.write("""\nQ values end \n\n""")'''


	return lengthm, rewardm



################################  Main    #####################################**

if __name__ == '__main__':
	try:
		rospy.init_node('q_learning', anonymous=True)
		now1 = rospy.get_rostime()
		t0 =  now1.secs%1000+round(now1.nsecs/10**6,0)*0.001
		print('time(sec.ms)',now1.secs%1000+round(now1.nsecs/10**6,0)*0.001)
		#rospy.sleep(4.)
		print('start_q_learning')
		now1 = rospy.get_rostime()
		print('now2.secs, now2.nsecs',now1.secs, now1.nsecs)
		print('now1.secs, now1.nsecs ms',now1.secs%1000+round(now1.nsecs/10**6,0)*0.001)
		
		outFileNameQ = '/home/ranjeet/catkin_ws/src/surveillance_pkg/nonmodular_programs/'+ 'q_values' + ".txt"
		outFile=open(outFileNameQ, "w+")
		outFile.write("""Q values start \n""")
		outFile.write(str(Q))
		outFile.write("""\n Q values end \n\n""")

		episode_lengths, episode_rewards = play_many_games(N)

		print('q_learning complete write')
		outFile.close()
  		plot_running_avg(episode_rewards)
		

	except rospy.ROSInterruptException:
		pass

