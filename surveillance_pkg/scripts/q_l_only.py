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
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import *
import random
from sensor_msgs.msg import BatteryState

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

observation = np.zeros(2)
pub = rospy.Publisher('action_topic', Float64)

def callback1(data):
	global v1
	v1 = (data.data)
    	return v1 

def callback2(data):
	global v2
	v2 = (data.data)
    	return v2


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
	print('change done')

	Q = {'02': {0: 0, 1: 1, 2: 0}, '03': {0: 0, 1: 1, 2: 0}, '00': {0: 0, 1: 1, 2: 0}, '01': {0: 0, 1: 1, 2: 0}, '20': {0: 0, 1: 0, 2: 1}, '21': {0: -12.511241332355644, 1: -50.95671541491758, 2: -38.724232709163616}, '22': {0: -36.61613718001919, 1: -24.18511231313147, 2: -36.69018044045552}, '10': {0: 0, 1: 0, 2: 1}, '33': {0: -9.209077061294401, 1: -38.43557127229569, 2: -38.05044729266092}, '32': {0: -14.85949821340816, 1: -27.041503280931966, 2: -36.477284238238454}, '31': {0: -39.7115599895896, 1: -69.27344559130378, 2: -28.652037260489244}, '23': {0: -14.964572112488666, 1: -37.42705789429603, 2: -28.576999780490695}, '13': {0: -47.00972048870312, 1: -28.63661185606952, 2: -80.92562000318863}, '12': {0: -12.336534316409235, 1: -5.3516011525931006, 2: -61.46038244751114}, '30': {0: 0, 1: 0, 2: 1}, '11': {0: -32.92409227881819, 1: -357.59016630950873, 2: -322.7382094722733}}

	act =  0
	act_prev = 0
	v1 = 100.0
	v2 = 100.0
	observation[0] = v1
	observation[1] = v2
	eflytime = []
	ereward = []
	emove2base = []
	episode_move_to_base = 0
	flytime_episode = 0
	episode_reward = 0
	for n in range(N):
		eps = 1/np.sqrt(n+1)
		i=0
		episode_length= 0
		done1 = True
		done2 = True
		done = False
		observation[0] = v1
		observation[1] = v2
		state = get_state_as_string(assign_bins(observation, bins))##########
		episode_reward = 0
		episode_move_to_base = 0

#------------------------------------------------   Greedy start -------------------------------------------------
		eps = 1/np.sqrt(n+1)
		i=0
		episode_length= 0
		done1 = True
		done2 = True
		done = False
		v1 = 100.0
		v2 = 100.0


		rospy_Rate= 1
		rate = rospy.Rate(rospy_Rate) 

		while not done:
			#print('rate:',rospy_Rate)

			while not rospy.is_shutdown():			
				i+=1

				observation[0] = v1
				observation[1] = v2
				episode_length += 1
				reward = 1

		 		rospy.Subscriber("uav0_battery", Float32, callback1)
				observation[0] = v1
				rospy.Subscriber("uav1_battery", Float32, callback2)
				observation[1] = v2

				print('i:',i,'v1:',v1, 'v2:',v2, 'act', act)

				state_new = get_state_as_string(assign_bins(observation, bins))#####

				if 0 < act:
					reward = -1
					episode_move_to_base += 1 

				if v1<25 and v2<25:
					reward = -1000
					flytime_episode = i
					done = True

				act_next = max_dict(Q[state_new])[0]
				pub.publish(act_next)
			    
				act_prev = act
				act = act_next
				state, act2 = state_new, act

				episode_reward += reward
				if done == True:
					print('done:',done, 'i:',i)
					break

				rate.sleep()

		eflytime.append(flytime_episode)
		ereward.append(episode_reward)
		emove2base.append(episode_move_to_base)

		print('iteration:',n)
		f.write("Q values start, episode = %d\n" % n)
		f.write("""flytime_episode = %d\n""" % flytime_episode)
		f.write("""episode_reward = %d\n""" % episode_reward)
		f.write("""total move to base = %d\n""" % episode_move_to_base)
		f.write(str(Q))
		f.write("""\nQ values end \n\n""")

#------------------------------------------------   Greedy end -------------------------------------------------


	return eflytime, ereward, emove2base




################################  Main    #####################################**

if __name__ == '__main__':

	try:
		rospy.init_node('q_learning', anonymous=True)
		outFileNameQ = '/home/ranjeet/catkin_ws/src/surveillance_pkg/nonmodular_programs/'+ 'q_values' + ".txt"# edit location
		f=open(outFileNameQ, "w+")
		f.write("""Q values start \n""")
		f.write("Woops! I have deleted the content!")
		f.write("""Q values start \n""")
		f.write(str(Q))
		f.write("""\n Q values end \n\n""")

		eflytime, ereward, emove2base = play_many_games(N)
		print('q_learning complete write')
		f.close()

		plot1 = plt.figure(1)
		N = len(eflytime)
		running_avg1 = np.empty(N)
		running_avg2 = np.empty(N)
		episode = np.empty(N)
		for t in range(N):
			episode[t] = t
			running_avg1[t] = np.mean(eflytime[max(0, t-10):(t+1)])
			running_avg2[t] = np.mean(emove2base[max(0, t-10):(t+1)])
		plt.plot(episode, running_avg1)
		plt.plot(episode, running_avg2)
		plt.title("flytime(max 1000), move2base_command_no vs episode")
		plt.legend(["flytime", " no of move2base"],  loc='upper left')
		#print('episode', episode,'running_avg1', running_avg1, 'running_avg2', running_avg2)
		plt.show()
		

	except rospy.ROSInterruptException:
		pass



