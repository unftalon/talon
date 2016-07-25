#!/usr/bin/env python
# imu_node.py this is a ROS node that publish raw IMU data.
# TODO: 
# 	-make it more intellegient so it will automatically known what port the tss is connected to.
# 	-figure out if it is necessary to use device.close() anywhere.

# written by Michael Otero
# email: mike at unftalon.org
# credit to some other authors that I will list here later.



import rospy
from sensor_msgs.msg import Imu
import sys
import time
from std_msgs.msg import String
import threespace as ts_api

# open the device
# TODO: make port assignment dynamic
device = ts_api.TSUSBSensor(com_port='/dev/ttyACM0')

# main function, should probably be called main.
def tss_node():

	# Set up a ROS publisher
	# Publisher('<Topic Name>', <Message Type>, queue_size=10)
	pub = rospy.Publisher('ImuData', Imu, queue_size=10)
	
	# Initialize the ROS node and name it.
	rospy.init_node('tss_node')

	# set the rate at which the loop should run in milliseconds
	r = rospy.Rate(10)
	
	
	while not rospy.is_shutdown():
		
		# get an instance of the Imu data type (phrasing?).
		imuMsg = Imu()
		
		# The data from the API comes in as a tuple.
		# here we convert the tuple to an array so each of its elements
		# can be assigned to an element of the Imu message.
		orientation = list(device.getTaredOrientationAsQuaternion())
		

		# Assign each element to a different part of the Imu message
		imuMsg.orientation.x = orientation[0]
		imuMsg.orientation.y = orientation[1]
		imuMsg.orientation.z = orientation[2]
		imuMsg.orientation.w = orientation[3]
		
		# Finally, publish the data
		pub.publish(imuMsg)
		
		# wait
		r.sleep()
		
	# spin once to keep this baby alive.
	rospy.spin()
	
if __name__ == '__main__':
	tss_node()


# do we need to close the port somewhere?