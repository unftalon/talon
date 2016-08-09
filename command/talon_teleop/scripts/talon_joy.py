# teleop_joy.py
# ROS node that will subscribe to the joy node and pass the information along
# as the proper data for interpretation.


#!/usr/bin/env python
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
	twist = Twist()
	
	# using the Twist type to get info from the joystick.
	twist.linear.x = joyToServoScale*data.axes[3] 
	pub.publish(twist)
	
 
def joy_teleop():
	# Initialize a ROS node
	rospy.init_node('Joy_teleop')
	
	# set up a subscriber of the topic "joy", type "Joy" and assign the callback function "callback"
	rospy.Subscriber("joy", Joy, callback)
	
	global pub # why "global pub"?
	
	# set up a publisher of topic "servo", type "Twist" and set a queue size
	pub = rospy.Publisher('servo', Twist, queue_size=10)
 
	# I don't remember what this is
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		r.sleep()
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
 
if __name__ == '__main__':
	joy_teleop()
