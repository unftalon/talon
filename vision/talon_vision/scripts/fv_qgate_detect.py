#!/usr/bin/env python
#
# fv_qgate_detect.py
# Forward Vision Qualification Gate Detector
# 
# This node will subscribe to filtered images and extract information about location of the qualification gate.
#
# TODO: 	- put the actual filters in a separate module?
#
# written by Michael Otero
# email: mike at unftalon.org
# credit to some other authors that I will list here later.

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def nothing(x):
	pass

class fv_qgate_detect:

	def __init__(self):
		# set up publisher and subscriber
		self.image_pub = rospy.Publisher("/vision/fwd/qgate_mask", Image, queue_size=1)
		self.image_sub = rospy.Subscriber("/vision/fwd/img_filtered", Image, self.callback)
		
		self.bridge = CvBridge()
		
		cv2.namedWindow('Trackbars')
		cv2.createTrackbar('lower_hue','Trackbars',0,179, nothing)
		cv2.createTrackbar('lower_sat','Trackbars',0,255, nothing)
		cv2.createTrackbar('lower_val','Trackbars',0,255, nothing)
		cv2.createTrackbar('upper_hue','Trackbars',0,179, nothing)
		cv2.createTrackbar('upper_sat','Trackbars',0,255, nothing)
		cv2.createTrackbar('upper_val','Trackbars',0,255, nothing)
		
	def callback(self, data):
	
		# try to convert the ROS image to a cv image and throw exception if it fails
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		hl = cv2.getTrackbarPos('lower_hue','Trackbars')
		sl = cv2.getTrackbarPos('lower_sat','Trackbars')
		vl = cv2.getTrackbarPos('lower_val','Trackbars')
		hu = cv2.getTrackbarPos('upper_hue','Trackbars')
		su = cv2.getTrackbarPos('upper_sat','Trackbars')
		vu = cv2.getTrackbarPos('upper_val','Trackbars')
		
		blur = cv2.GaussianBlur(cv_image, (5,5),0)
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		
		''' so far it looks like the red ball has the following upper and lower limits for HSV:
		
			lower: 4,150,130
			upper: 15,230,243
			
		'''
		lower_green = np.array([hl,sl,vl])
		upper_green = np.array([hu,su,vu])
		
		# threshold the HSV image to get only desired color
		mask = cv2.inRange(hsv, lower_green, upper_green)
		
		bmask = cv2.GaussianBlur(mask, (5,5),0)
		
		res = cv2.bitwise_and(cv_image,cv_image, mask=mask)
		
		
		

		# create a window in the GUI to show the cv image
		cv2.imshow("Image Window", res)
		cv2.waitKey(3)		
			
			
			
def main(args):
	fqd = fv_qgate_detect()
	rospy.init_node('fv_qgate_detect', anonymous=True)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
	cv2.destroyAllWindows()

	
if __name__=='__main__':
	main(sys.argv)