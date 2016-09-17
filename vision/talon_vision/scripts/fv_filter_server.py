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
		self.image_pub = rospy.Publisher("/vision/fwd/img_filtered", Image, queue_size=1)
		
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
		
	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		# do some things hewre
		(rows,cols,channels) = cv_image.shape
		if cols > 60 and rows > 60 :
			cv2.circle(cv_image, (50,50), 10, 255)
			
		
		cv2.imshow("Image Window", cv_image)
		cv2.waitKey(3)
		
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