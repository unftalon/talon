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
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# I don't remember why this is here, perhaps its nothing?
def nothing(x):
	pass

class fv_qgate_detect:

	def __init__(self):
		# set up publishers and subscribers
		self.image_pub = rospy.Publisher("/vision/fwd/qgate_mask", Image, queue_size=1)
		self.image_sub = rospy.Subscriber("/vision/fwd/img_filtered", Image, self.callback)
		self.coord_pub = rospy.Publisher("/vision/fwd/centerCoords", Pose, queue_size=10)
		
		# instantiate CvBridge
		self.bridge = CvBridge()
		
		# create OpenCV GUI trackbars
		# TODO: learn what the "nothing" part does
		cv2.namedWindow('Trackbars')
		cv2.createTrackbar('lower_hue','Trackbars',0,179, nothing)
		cv2.createTrackbar('lower_sat','Trackbars',0,255, nothing)
		cv2.createTrackbar('lower_val','Trackbars',0,255, nothing)
		cv2.createTrackbar('upper_hue','Trackbars',0,179, nothing)
		cv2.createTrackbar('upper_sat','Trackbars',0,255, nothing)
		cv2.createTrackbar('upper_val','Trackbars',0,255, nothing)
		
	# callback function is called every time information comes in from the img_filtered topic
	def callback(self, data):
	
		# try to convert the ROS image to a cv image and throw exception if it fails
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		# get the trackbar pos for each value and set it to respective variables
		hl = cv2.getTrackbarPos('lower_hue','Trackbars')
		sl = cv2.getTrackbarPos('lower_sat','Trackbars')
		vl = cv2.getTrackbarPos('lower_val','Trackbars')
		hu = cv2.getTrackbarPos('upper_hue','Trackbars')
		su = cv2.getTrackbarPos('upper_sat','Trackbars')
		vu = cv2.getTrackbarPos('upper_val','Trackbars')
		
		# blur the image to reduce noise
		blur = cv2.GaussianBlur(cv_image, (5,5),0)
		
		# convert the color space from BGR to HSV
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
		
		''' so far it looks like the red ball has the following upper and lower limits for HSV:
		
			lower: 4,150,130
			upper: 15,230,243
			
			here's the green sticky note stuck to my desk area
			
			lower: 24, 111, 173
			upper: 57, 208, 255
			
		'''
		
		# assign the lower and upper ranges for the mask
		lower_green = np.array([hl,sl,vl])
		upper_green = np.array([hu,su,vu])
		
		# threshold the HSV image to get only desired color, this creates a 'mask'
		mask = cv2.inRange(hsv, lower_green, upper_green)
		
		# blur the mask to reduce noise
		# NOTE: i think this got lost and is currently not being used (fix this)
		bmask = cv2.GaussianBlur(mask, (5,5),0)
		
		# recomibine the pre-mask image and the positive values for the mask so that
		# the original image appears through the mask (easier to see what is happening)
		res = cv2.bitwise_and(cv_image,cv_image, mask=mask)
		res = cv2.erode(res, None, iterations = 2)
		res = cv2.dilate(res, None, iterations = 2)
		
		
		###================================================= experimental code below
		# see http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/ for source of below code. will be
		# beter explained later
		# the below variables and things should be adjusted to fit into this overall code better. they are
		# copy pasted here and don't flow with everything else. 
		
		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
 
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
			if radius > 10:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(cv_image, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
				cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
		else:
			center = (-1,-1)

		###================================================= experimental code above
		
		# create a window in the GUI to show the cv image
		# NOTE: this should be removed when fully implemented into ROS. output should be published and viewed with rqt
		cv2.imshow("Image Window", cv_image)
		cv2.imshow("Mask window", res)
		cv2.waitKey(3)
		
		
		# instantiate Pose
		coord = Pose()
		
		# assign values of coord to the coordinates
		coord.position.x = center[0]
		coord.position.y = center[1]
		
		# publish the coordinates of the circle that was generated
		self.coord_pub.publish(coord)
		
		# try to convert the cv image to a ROS image and throw exception if it fails	
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)
			
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