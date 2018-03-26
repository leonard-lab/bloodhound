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

last_keypress = ''
tag_name = ''
last_tag = TransformStamped()

def callback_keyboard(data):
    global last_keypress 
    last_keypress = data.data

def callback_apriltags(data):
	#Read in data from apriltags and save
	global last_tag
	last_tag = data.transforms[0]
	#tag_name = data.transforms[0].child_frame_id
	#print data.transforms[0].transform.translation.x



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

    #Subscribe to topics
    rospy.Subscriber('keyboard', String, callback_keyboard)
    rospy.Subscriber('tf', TFMessage, callback_apriltags)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():


    	if last_keypress == 'm':
    		print "m key pressed"
	    	#Clear last keypress once read
	    	last_keypress = ''
	    	print last_tag.child_frame_id


	    	#Open Text File for writing
	    	data_path = "/home/leonardlab/Documents/TestSiteData/"
	    	text_file = open(data_path + "SiteMeasurement" + last_tag.child_frame_id+".txt", "w")

	

        rate.sleep()

if __name__ == '__main__':
    try:
        sitedegreemeasurement()
    except rospy.ROSInterruptException: 
        global text_file
        text_file.close()
