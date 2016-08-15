#!/usr/bin/env python

import rospy
import sys
import time
import numpy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from talon_pid import PID





# this needs to be moved to some common library somewhere
# it quickly breaks a message into an array
xyz_array = lambda o: numpy.array([o.x, o.y, o.z])
xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])

rospy.init_node('talon_controller', anonymous=False)

class talon_control:

	def __init__(self):
		# Create arrays to be used
		self.desired_state = numpy.ones(6)
		self.desired_velocity = numpy.ones(6)
		self.current_state = numpy.zeros(6)
		self.current_velocity = numpy.zeros(6)
		self.current_error = numpy.ones(0)
		
		# ROS components
		rospy.Subscriber("odom", String, self.odom_cb, queue_size=1)
		rospy.Subscriber("trajectory", Pose, self.trajectory_cb, queue_size=1)
		self.controller_wrench = rospy.Publisher("wrench", Pose, queue_size=1)


	def odom_cb(self, msg):
		'''
		this is for getting the odometry data on subscribe
		'''
		
		rospy.loginfo(rospy.get_caller_id() + "I head %s", msg.data)
		
	def trajectory_cb(self, desired_trajectory):
		'''
		this is the callback for the trajectory subscription
		'''
		# Get desired pose and orientation
		
		self.desired_pose = xyz_array(desired_trajectory.position) # desired pose
		self.desired_orientation = xyzw_array(desired_trajectory.orientation) #desired orientation

		
		
	def talon_controller(self, event):
		'''
		this is the main for talon_controller node
		'''
		

		

		
		# this is our main thing
		
		# calculate errors
		
		# combine errors into the same array
		
		# send errors through PID controller
		
		# combine into final wrench (for output)
		
		# publish values
		#print self.desired_pose
		self.controller_wrench.publish(desired_pose)
		
		
		
if __name__ == '__main__':
	controller = talon_control()
	rospy.Timer(rospy.Duration(1.0/50.0), controller.talon_controller)
	rospy.spin()