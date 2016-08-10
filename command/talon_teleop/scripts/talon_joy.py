#!/usr/bin/env python
# talon_teleop.py
# ROS node that will subscribe to the joy node and pass the information along
# as the proper data for interpretation.



import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import sys
import time

# this scales the joy input to a larger number.
# for the case of arduino servo.h, set to 180 so it can be mapped to
# the expected servo input of 0-180 (angle)
# there is probably a better way to do this.
joyToServoScale = 180
 
# define the callback function to be called when something comes in from the subscription
def callback(data):
	cmd = Twist()
	
	# using the Twist type to get info from the joystick.
	# z = heave
	# axes[3] is the flipper on the logitech joystick
	cmd.linear.z = joyToServoScale*data.axes[3] 
	pub.publish(cmd)
	
 
def talon_teleop():
	# Initialize a ROS node
	rospy.init_node('talon_teleop')
	
	# set up a subscriber of the topic "joy", type "Joy" and assign the callback function "callback"
	rospy.Subscriber("joy", Joy, callback)
	
	# set up a publisher of topic "servo", type "Twist" and set a queue size
	global pub
	pub = rospy.Publisher('thrust', Twist, queue_size=10)
 
	# reduce the rate of the loop and keep it alive
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		r.sleep()
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
 
if __name__ == '__main__':
	talon_teleop()
