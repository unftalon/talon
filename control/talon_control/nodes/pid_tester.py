#!/usr/bin/env python
# for testing the talon_pid node



import rospy
from geometry_msgs.msg import Pose
import sys
import time
	
 
def pid_tester():
	# Initialize a ROS node
	rospy.init_node('pid_tester')

	pub = rospy.Publisher('trajectory', Pose, queue_size=10)
	print "PID Tester Started."
	
	
 
	# reduce the rate of the loop and keep it alive
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		r.sleep()
		
		d = Pose()
		d.position.x = 1
		d.position.y = 2
		d.position.z = 3
		d.orientation.x = 4
		d.orientation.y = 5
		d.orientation.z = 6
		d.orientation.w = 7
		pub.publish(d)
		
		
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
 
if __name__ == '__main__':
	pid_tester()

