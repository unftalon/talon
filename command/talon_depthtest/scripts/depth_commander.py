#!/usr/bin/env python
#
# depth_commander.py
# Depth commander node
# 
# This node will subscribe to a commanded depth from various places, calculate
# the error between it and the actual depth, and send the appropriate commands to a thruster
# commander (maybe)
#
# TODO: 	
#
# written by Michael Otero
# email: mike at unftalon.org
# 

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import FluidPressure
import sys
import time



class depth_commander:
	
	def __init__(self):
		
		rospy.Subscriber("/command/pose", Pose, trajectory_cb)
		rospy.Subscriber("/sensors/pressure", FluidPressure, pressure_cb)
		rospy.Publisher('/command/wrench', Wrench, queue_size=10)
 
	# define the callback function to be called when something comes in from the subscription
	def trajectory_cb(self, data):
		trajectory = Pose()
		# set the desired depth from the trajectory topic to a global variable
		self.desired_depth = 1 #trajectory.position.z
		
		
	def pressure_cb(self, data):
		# convert the pressure to depth
		# get commanded depth
		# get current depth
		# send to PID function
		# get returned value
		pressure = FluidPressure()
		
		self.current_depth = pressure.fluid_pressure

	#def PID(self, current_depth, desired_depth):
	def PID(self, current_depth, desired_depth):
		farts = 2
		
	def main_loop(self, event):
	
		foo = self.desired_depth
		print(foo)
	

def main(args):
	rospy.init_node('depth_commander')
	try:
		print("mainthing")
	except KeyboardInterrupt:
		print("Depth Commander shutting down...")
		
 
if __name__ == '__main__':
	#main(sys.argv)
	#rospy.spin()

	commander = depth_commander()
    #rospy.on_shutdown(controller.shutdown)
	rospy.Timer(rospy.Duration(1.0/50.0), commander.main_loop)
    #rospy.Timer(rospy.Duration(1), controller.timeout_callback)
	rospy.spin()