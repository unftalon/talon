#!/usr/bin/env python
#
# fv_raw_server.py
# Forward Vision Raw Image Server
# 
# This node will get the raw images from the camera driver and publish them
#
# TODO: 
#    - give it a real name
#    - make any image filtering settings parametrized or adjustable on the fly
# 	
#
# written by Michael Otero
# email: mike at unftalon.org
# credit to some other authors that I will list here later. (uf-mil)

import sys
import rospy
import time