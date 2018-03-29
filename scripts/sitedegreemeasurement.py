#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Peter Landgren.
# All rights reserved.
#
## Bloodhound site measurement code

#Dependencies
import rospy
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
import math
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler

last_keypress = ''
tag_name = ''
last_tag = Twist()
last_tag_time = time.time()
on_target_flag = False
on_target_time = 0;

def callback_keyboard(data):
    global last_keypress 
    last_keypress = data.data

def callback_apriltags(data):
	#Read in data from apriltags and save
	global last_tag
	global last_tag_time
	global tag_name
	transform_temp = data.transforms[0]
	[last_tag.angular.x, last_tag.angular.y, last_tag.angular.z] = euler_from_quaternion([transform_temp.transform.rotation.x, transform_temp.transform.rotation.y, transform_temp.transform.rotation.z, transform_temp.transform.rotation.w])
	[last_tag.linear.x, last_tag.linear.y, last_tag.linear.z] = [transform_temp.transform.translation.x, transform_temp.transform.translation.y, transform_temp.transform.translation.z]
	last_tag_time  = time.time()
	tag_name = data.transforms[0].child_frame_id
	#print data.transforms[0].transform.translation.x
	#print "Last Tag"
	#print last_tag

def control_law(current_pos, target_pos, last_measurement_time, tolerances):
	#Calculate control input for robot
	#Global variables
	global on_target_flag
	global on_target_time

	#proportional control gains
	k_x = 1
	k_y = 1
	k_z = 1

	#max values
	max_vel = Twist()
	max_vel.linear.x = 0.2;
	max_vel.linear.y = 0.2;
	max_vel.angular.z = 0.2;

	#constant offsets
	const_offset = Twist()
	const_offset.linear.x = 0.005
	const_offset.linear.y = 0.005
	const_offset.angular.z = 0.005

	#Differences in pos
	diff = Twist()
	diff.linear.x = target_pos.linear.x - current_pos.linear.x
	diff.linear.y = target_pos.linear.y - current_pos.linear.y
	temp = wraptopi([target_pos.angular.z - current_pos.angular.z])
	diff.angular.z = temp[0]
	on_target_flag_last = on_target_flag
	on_target_flag = False

	commands = Twist()
	timeout = 0.5;
	if time.time() - last_measurement_time > timeout:
		return commands

	on_target_flag = True 
	if abs(diff.linear.x) > tolerances.linear.x:
		commands.linear.y = min(abs((diff.linear.x) * k_x), max_vel.linear.x) * np.sign(diff.linear.x)
		on_target_flag = False

	if abs(diff.linear.y) > tolerances.linear.y:
		commands.linear.x = min(abs((diff.linear.y) * k_y), max_vel.linear.y) * np.sign(diff.linear.y)
		on_target_flag = False

	if abs(diff.angular.z) > tolerances.angular.z:
		commands.angular.z = min(abs((diff.angular.z) * k_z), max_vel.angular.z) * np.sign(diff.angular.z) 
		on_target_flag = False
	#print diff

	#Set on_target_time
	if on_target_flag == True and on_target_flag_last == False:
		on_target_time= time.time()
		rospy.loginfo("On Angle")

	return commands

def wraptopi(angle):
	newAngleVec = angle
	for i in range(0,len(angle)):
		#Wraps angle to -180 to 180
		while (newAngleVec[i]<= -math.pi):
			newAngleVec[i] = newAngleVec[i] + 2.0 * math.pi
		while (newAngleVec[i] > math.pi):
			newAngleVec[i] = newAngleVec[i]- 2.0 * math.pi
	return newAngleVec

def wrapto2pi(angle):
	newAngleVec = angle
	for i in range(0,len(angle)):
		#Wraps angle to -180 to 180
		while (newAngleVec[i]<= 0):
			newAngleVec[i] = newAngleVec[i] + 2.0 * math.pi
		while (newAngleVec[i] > 2.0*math.pi):
			newAngleVec[i] = newAngleVec[i]- 2.0 * math.pi
	return newAngleVec

def wrapto180(angle):
	newAngleVec = angle
	for i in range(0,len(angle)):
		#Wraps angle to -180 to 180
		while (newAngleVec[i]<= -180):
			newAngleVec[i] = newAngleVec[i] + 360
		while (newAngleVec[i] > 180):
			newAngleVec[i] = newAngleVec[i]- 360
	return newAngleVec

#Main Code
def sitedegreemeasurement():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	rospy.init_node('sitedegreemeasurement', anonymous=True)

	#Global Variables
	global last_keypress
	global last_tag
	global last_tag_time
	global tag_name
	global on_target_flag
	global on_target_time

	#Subscribe to topics
	rospy.Subscriber('keyboard', String, callback_keyboard)
	rospy.Subscriber('tf', TFMessage, callback_apriltags)

	#Publish topics
	pub_motorcmd = rospy.Publisher('/bloodhound/motorcmd', Twist, queue_size=10)

	running_flag = False
	d_angle = 30
	site_angles = wrapto180(np.linspace(0,360,360/d_angle,endpoint=False))
	tolerances = Twist()
	tolerances.linear.x = 0.001
	tolerances.linear.y = 0.001
	tolerances.angular.z = 0.001


	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():


		if last_keypress == 'm':
			rospy.loginfo("Beginning Site Measurement")
			#Clear last keypress once read
			last_keypress = ''
			print tag_name


			#Open Text File for writing
			data_path = "/home/leonardlab/Documents/TestSiteData/"
			text_file = open(data_path + "SiteMeasurement" + tag_name +".txt", "w")

			#Start running_flag
			running_flag = True

			#Start sample counter
			sample_cnt = 0;

		if last_keypress == 'c':
			rospy.loginfo("Clearing Measurement Commands")
			#Clear last keypress once read
			last_keypress = ''

			#Stop running_flag
			running_flag = False

			#Exit text file
			try:
				text_file.close()
			except:
				pass


		if running_flag:
			#If running do a measurement run
			stay_time = 5 #Number of seconds to stay at each location
			target_pos = Twist();
			target_pos.linear.x = -0.006
			target_pos.linear.y = 0.005
			target_pos.angular.z = site_angles[sample_cnt] / 360.0 * 2.0 * math.pi
			pub_motorcmd.publish(control_law(last_tag, target_pos, last_tag_time, tolerances))

			#print "Time"
			#print time.time()
			#print on_target_time

			if on_target_flag:
				if time.time() - on_target_time > stay_time:
					#Make measurement
					sample_cnt = sample_cnt + 1;
					rospy.loginfo("Sampling angle %i" %(site_angles[sample_cnt]))
					on_target_flag = False

					if sample_cnt >= len(site_angles) - 1:
						#Site finished, close text_file and stop
						running_flag = False
						try:
							text_file.close()
						except:
							pass

			

	

		rate.sleep()

if __name__ == '__main__':
	try:
		sitedegreemeasurement()
	except rospy.ROSInterruptException: 
		pub_motorcmd.publish(Twist())
		global text_file
		text_file.close()
