#!/usr/bin/env python
# tss_test2.py this is a ROS node that will make sure everything is working properly.
# TODO: make it more intellegient so it will automatically known 
# what port the tss is connected to.



import rospy
from sensor_msgs.msg import Imu
import sys
import time
from std_msgs.msg import String
import threespace as ts_api

device = ts_api.TSUSBSensor(com_port='/dev/ttyACM0')

def tss_node():
	# Initialize a ROS node
	pub = rospy.Publisher('ImuData', Imu, queue_size=10)
	rospy.init_node('tss_node')

	
	
	
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		#ledTup = device.getLEDColor()
		#led = list(ledTup)
		imuMsg = Imu()
		orientation = list(device.getTaredOrientationAsQuaternion())
		

		
		imuMsg.orientation.x = orientation[0]
		imuMsg.orientation.y = orientation[1]
		imuMsg.orientation.z = orientation[2]
		imuMsg.orientation.w = orientation[3]
		
		pub.publish(imuMsg)
		
		
		#print(led)
		#pub.publish(led)
		r.sleep()
		
	rospy.spin()
	
if __name__ == '__main__':
	tss_node()


#if device is not None:
    ## Now we can start getting information from the device.
    ## The class instances have all of the functionality that corresponds to the
    ## 3-Space Sensor device type it is representing.
    #print("It looks like we've connected.")
    #print("==================================================")
    #print("Getting the filtered tared quaternion orientation.")
    #quat = device.getTaredOrientationAsQuaternion()
    #if quat is not None:
    #    print(quat)
    #print("==================================================")
    #print("Getting the raw sensor data.")
    #data = device.getAllRawComponentSensorData()
    #if data is not None:
    #    print("[%f, %f, %f] --Gyro\n"
    #          "[%f, %f, %f] --Accel\n"
    #          "[%f, %f, %f] --Comp" % data)
    #print("==================================================")
    #print("Getting the LED color of the device.")
    #led = device.getLEDColor()
    #if led is not None:
    #    print(led)
    #print("==================================================")
    
    ## Now close the port.
    
