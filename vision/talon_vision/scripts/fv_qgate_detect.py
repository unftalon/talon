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

class fv_qgate_detect:

	def __init__(self):
		# set up publisher and subscriber
		self.image_pub = rospy.Publisher("/vision/fwd/qgate_mask", Image, queue_size=1)
		self.image_sub = rospy.Subscriber("/vision/fwd/img_filtered", Image, self.callback)
		
		self.bridge = CvBridge()
		
	def callback(self, data):
	
		# try to convert the ROS image to a cv image and throw exception if it fails
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
			
		blur = cv2.GaussianBlur(cv_image, (5,5),0)
		
		# create a window in the GUI to show the cv image
		cv2.imshow("Image Window", blur)
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