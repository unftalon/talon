#!/usr/bin/env python

import rospy
import sys
import time
from std_msgs.msg import String

def odom_cb(msg):
	'''
	this is for getting the odometry data on subscribe
	'''
	
	rospy.loginfo(rospy.get_caller_id() + "I head %s", msg.data)
	
def trajectory_cb(msg):
	'''
	this is the callback for the trajectory subscription
	'''
	
def talon_controller():
	'''
	this is the main for talon_controller node
	'''
	
	rospy.init_node('talon_controller', anonymous=False)
	
	rospy.Subscriber("odom", String, odom_cb, queue_size=1)
	rospy.Subscriber("trajectory", String, trajectory_cb, queue_size=1)
	rospy.Publisher("wrench", String, queue_size=1)
	
	rospy.spin()
	
if __name__ == '__main__':
	talon_controller()