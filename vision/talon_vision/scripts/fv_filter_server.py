#!/usr/bin/env python
#
# fv_filter_server.py
# Forward Vision Filter Server
# 
# This node will subscribe to raw camera imgages and apply filters for making them
# easier to read by other services.
#
# TODO: 	
#
# written by Michael Otero
# email: mike at unftalon.org
# credit to some other authors that I will list here later. (uf-mil)

import sys
import rospy
import cv2 #NEED TO SET THIS UP
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # NEED TO SET THIS UP TOO

class fv_filter_server:
	
	def __init__(self):
		# set up publisher and subscriber
		self.image_pub = rospy.Publisher("/vision/fwd/img_filtered", Image, queue_size=1)
		self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
		
		# instantiate the CvBridge
		self.bridge = CvBridge()
		
	# callback function for subscriber.
	def callback(self, data):
	
		# try to convert the ROS image to a cv image and throw exception if it fails
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		# this is a test modification. draws some circle on the top left of the image
		(rows,cols,channels) = cv_image.shape
		if cols > 60 and rows > 60 :
			cv2.circle(cv_image, (50,50), 10, 255)
			
		# create a window in the GUI to show the cv image
		cv2.imshow("Image Window", cv_image)
		cv2.waitKey(3)
		
		# try to convert the image back to ROS format and publish it.
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

def main(args):
	ffs = fv_filter_server()
	rospy.init_node('fv_filter_server', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
	cv2.destroyAllWindows()
	
if __name__=='__main__':
	main(sys.argv)