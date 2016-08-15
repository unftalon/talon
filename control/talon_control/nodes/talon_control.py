#!/usr/bin/env python

import rospy
import sys
import time

def odom_cb(msg):
	'''
	this is for getting the odometry data on subscribe
	'''
	
def trajectory_cb(msg):
	'''
	this is the callback for the trajectory subscription
	'''
	
def talon_controller():
	'''
	this is the main for talon_controller node
	'''
	
	rospy.init_node('talon_controller', anonymous=False)
	
	rospy.Subscriber("odom", String, odom_cb)
	rospy.Subscriber("trajectory", String, trajectory_cb)
	
	rospy.spin()
	
if __name__ == '__main__':
	talon_controller():