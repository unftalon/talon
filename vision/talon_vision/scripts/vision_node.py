#!/usr/bin/env python
# camera vision node
# TODO: 
#    - give it a real name
#    - make any image filtering settings parametrized or adjustable on the fly
# 	
# the goal of this node is to receive the raw image from the camera driver,
# convert it to openCV format, apply any image filtering things,
# then publish
#
# written by Michael Otero
# email: mike at unftalon.org
# credit to some other authors that I will list here later. (uf-mil)

import sys
import rospy
import cv2 #NEED TO SET THIS UP
from cv_bridge import CvBridge, CvBridgeError # NEED TO SET THIS UP TOO
import numpy as np
import time


class vision_node:
    
    def __init__(self):
        # setup pubs and subs here
        
        # setuping (wtf i made a new word) publisher for image
        # that has been converted to openCV from ROS
        self.cv_image = rospy.Publisher("cv_image", Image, queue_size=1)
        
        # here we will subscribe to the raw images from the camera driver
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        
    def callback(self, data):
        #here's the callback
        
        # need to come back and understand what this means
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError, e:
            print e
        
# the below code is taken from uf_mil/propagator/camera_docking
def main(args):
    rospy.init_node('vision_node')
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    #something else might need to go here
    rospy.spin()