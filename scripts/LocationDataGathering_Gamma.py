#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Gamma bloodhound gather data from one locaiton

#Dependencies
import rospy
from std_msgs.msg import String, Bool, Float32, ByteMultiArray, UInt32
from geometry_msgs.msg import Twist, TransformStamped
from kobuki_msgs.msg import BumperEvent
import numpy as np
import math
import time

#Variables
global main_rate

main_rate = 10
global secs_at_target
global d_angle
secs_at_target = 3
d_angle = 5
global on_target_flag
on_target_flag = False

global current_angle #Current angle in degrees from 0 to 360
current_angle = 0 #Initialize current angle

global deg_to_rotate #Number of degrees to rotate
deg_to_rotate = 120

global text_file_Raw
text_file_Raw = open("RawDetections.txt", "w")
text_file_Raw.write("python time, arduino time, detector number \n")



#Define callbacks for all subscribers
def callback_odom(data):
    global current_angle
    current_angle = wrapto360(np.rad2deg(data.transform.rotation.z)) 

def callback_gamma1(data):
    global current_angle
    global text_file_Raw
    global on_target_flag

    if on_target_flag:
        text_file_Raw.write("%i, %i \n" %(time.time(), data, round(current_angle), 1))

def callback_gamma2(data):
    global current_angle
    global text_file_Raw
    global on_target_flag

    if on_target_flag:
        text_file_Raw.write("%i, %i \n" %(time.time(), data, round(current_angle), 2))

def callback_gamma3(data):
    global current_angle
    global text_file_Raw
    global on_target_flag

    if on_target_flag:
        text_file_Raw.write("%i, %i \n" %(time.time(), data, round(current_angle), 3))

#General Functions

def create_Rotation_Matrix(angle_Value):
    #angle_Value: angle ~ [0,2*pi)
    #return: 2x2 matrix

    ccw_Rotation_Matrix = np.matrix([[np.cos(angle_Value), -1*np.sin(angle_Value)],[np.sin(angle_Value), np.cos(angle_Value)]])
    return ccw_Rotation_Matrix


def Control_Law(current_target_angle):
    global current_angle
    global on_target_flag
    global target_region
    velocity_command = Twist()

    if abs(current_angle - current_target_angle) < target_region:
        on_target_flag = True
        return velocity_command
    else:
        on_target_flag = False
        velocity_command.angular.z = 0.075 * np.sign(wrapto180(current_angle - current_target_angle))  #bang/bang control

    return velocity_command


def get_target_angle(angle_time_spent):
    global secs_at_target

    index = -1

    for i in range(0,len(angle_time_spent)):
        if secs_at_target > angle_time_spent[i]:
            index = i
            break
    return index

def wrapto360(self, angle):
    #Wraps angle to -0 to 360
    newAngle = angle;
    while (newAngle < 0):
        newAngle = newAngle + 360
    while (newAngle >= 360):
        newAngle = newAngle - 360;
    return newAngle;

def wrapto180(angle):
    #Wraps angle to -180 to 180
    newAngle = angle;
    while (newAngle <= -180):
        newAngle = newAngle + 360
    while (newAngle > 180):
        newAngle = newAngle - 360;
    return newAngle;


#Main Code
def LocationDataGathering():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('LocationDataGathering', anonymous=True)
    #global variables
    global secs_at_target
    global d_angle
    global main_rate
    global current_angle
    global on_target_flag
    global target_region
    global deg_to_rotate


    #Angle code
    angles = np.arange(0, deg_to_rotate-d_angle, d_angle)
    angle_time_spent = np.zeros(len(angles))

    current_target_angle_i = 0
    target_region = .1


    #Subscribe to topics
    rospy.Subscriber('/odom', TransformStamped, callback_odom)
    rospy.Subscriber('/gamma1', UInt32, callback_gamma1)
    rospy.Subscriber('/gamma2', UInt32, callback_gamma2)
    rospy.Subscriber('/gamma3', UInt32, callback_gamma3)

    #Set up publishers
    pub_velocity_command = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    time1= time.time()
    # Keep node going until it is stopped
    rate = rospy.Rate(main_rate) # 10hz
    while not rospy.is_shutdown():



        time2 = time1
        time1= time.time()

        #If on target, add time in to time
        if on_target_flag:
            angle_time_spent[current_target_angle_i] += time1-time2
        
        current_target_angle_i_old = current_target_angle_i
        current_target_angle_i = get_target_angle(angle_time_spent)

        if current_target_angle_i != current_target_angle_i_old:
            print ("Moving to angle : %i" %(angles[current_target_angle_i]))

        if current_target_angle_i == -1: #Exit if all angles sampled enough
            pub_velocity_command.publish(Twist())
            print "Location Data Collection Completed"
            text_file_Raw.close()
            exit()
        else:
            current_target_angle = angles[current_target_angle_i]

            velocity_command = Control_Law(current_target_angle)
            pub_velocity_command.publish(velocity_command)


        rate.sleep()

if __name__ == '__main__':
    try:
        LocationDataGathering()
    except rospy.ROSInterruptException: 
        global text_file_Raw
        text_file_Raw.close()
