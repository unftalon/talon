# tss_test2.py this is a ROS node that will make sure everything is working properly.
# TODO: make it more intellegient so it will automatically known 
# what port the tss is connected to.
#!/usr/bin/env python2


import rospy
# import sensor_msgs IMU
import sys
import time

def callback(data):
		
		
def tss_node():
	# Initialize a ROS node
	rospy.init_node('tss_node')
	
	# set up a subscriber of the topic "imu", type "Imu" and assign the callback function
	rospy.Subscriber("imu", Imu, callback)
	
	global pub
	pub = rospy.Publisher('imuData', Imu, queue_size=10)
	
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()
		
	rospy.spin()
	
if __name__ == '__main__':
	tss_node()