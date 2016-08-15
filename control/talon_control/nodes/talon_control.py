#!/usr/bin/env python

import rospy
import sys
import time
from std_msgs.msg import String

from talon_pid import PID

def odom_cb(msg):
	'''
	this is for getting the odometry data on subscribe
	'''
	
	rospy.loginfo(rospy.get_caller_id() + "I head %s", msg.data)
	
def trajectory_cb(msg):
	'''
	this is the callback for the trajectory subscription
	'''
	
	# get desired trajectory from msg
	# assign to variable
	
def talon_controller():
	'''
	this is the main for talon_controller node
	'''
	
	rospy.init_node('talon_controller', anonymous=False)
	
	rospy.Subscriber("odom", String, odom_cb, queue_size=1)
	rospy.Subscriber("trajectory", String, trajectory_cb, queue_size=1)
	rospy.Publisher("wrench", String, queue_size=1)
	
	
	
	# this is our main thing
	
	# calculate errors
	
	# combine errors into the same array
	
	# send errors through PID controller
	
	# combine into final wrench (for output)
	
	# publish values
	
	rospy.spin()
	
if __name__ == '__main__':
	talon_controller()