#!/usr/bin/env python

import sys
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from time import sleep
from std_msgs.msg import Empty

DO_DEBUG=True
#
IMAGE_TOPIC_SUB = 'semantic_color'
DEBUG_IMAGE_TOPIC_PUB = 'debug_sem_centroid'
#
NUM_ERODE_DILATIONS = 10
#
WIDTH_KEEPING_FOV = 60 # pixels
DISTANCE_KEEP_AWAY = 200 # minEnclosingCircle radius
DISTANCE_KEEPING_RANGE = 20 # minEnclosingCircle radius

class FollowHuman:
    def __init__(self):
	rospy.loginfo('FollowHuman class constructor')

	# Image
	self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC_SUB, Image, self.image_cb, queue_size=1)
	self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC_PUB, Image, queue_size=1)

	# Drone
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


    def image_cb(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data)

	i_height, i_width, _ = img.shape


	img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	img_gray = cv2.erode(img_gray, None, iterations=NUM_ERODE_DILATIONS)
	img_gray = cv2.dilate(img_gray, None, iterations=NUM_ERODE_DILATIONS)
	cnts = cv2.findContours(img_gray.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]
	#print(len(cnts))

	if len(cnts) > 0:
	    c = max(cnts, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    X = int(x)
	    Y = int(y)
	    #print(radius)
	    if radius > 10:

		# move yaw to center human
		if X < (i_width/2 - WIDTH_KEEPING_FOV):
		    print('turn ccw')
		    self.move(0,0,0.1)
		elif X > (i_width/2 + WIDTH_KEEPING_FOV):
		    print('turn cw')
		    self.move(0,0,-0.1)
		else: 
		    # make sure drone is locked on when it moves towards or away
		    if radius < (DISTANCE_KEEP_AWAY-DISTANCE_KEEPING_RANGE/2):
		        print('move forward')
		        self.move(0.1,0,0) 
		    elif radius > (DISTANCE_KEEP_AWAY+DISTANCE_KEEPING_RANGE/2):
		        print('move back')
		        self.move(-0.1,0,0) 
		    else:
			self.move(0,0,0)
			print('do nothing')

		if DO_DEBUG:	
	            font = cv2.FONT_HERSHEY_SIMPLEX
	            cv2.putText(img,'Target Detected',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)
		    cv2.rectangle(img,(int(x-radius),int(y+radius)),
				  (int(x+radius),int(y-radius)),(255,255,255),1)
		    cv2.rectangle(img,(int(x-5),int(y+5)),(int(x+5),int(y-5)),(255,255,255),1)
       
	else: # no humans found
	    self.move(0,0,0.2) # turn until you find a human
	    print('turn to find human')

	if DO_DEBUG:
	     self.debug_image_pub.publish( self.cv_bridge.cv2_to_imgmsg(img, encoding='rgb8') )

    def move(self, lx, lz, az):
	twist = Twist()

	twist.linear.x =  lx  # forward +x, backward -x 
	twist.linear.z =  lz  # up +z, down -z
	twist.angular.z = az  # ccw +yaw, cw -yaw

	self._cmd_vel_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node('follow_human')
    f = FollowHuman()
    rospy.spin()

    # test
    if 0:
	    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	    takeoff_pub = rospy.Publisher('takeoff', Empty, queue_size=1)
	    land_pub = rospy.Publisher('land', Empty, queue_size=1)
	    sleep(1)
	    rospy.loginfo('Setup Publishers')

	    takeoff_pub.publish()
	    for i in range(10):
		sleep(0.5)
		cmd_vel_pub.publish(Twist(angular=Vector3(0,0,0.5)))
	    cmd_vel_pub.publish(Twist())
	    sleep(0.5)
	    land_pub.publish()
	    rospy.loginfo('Published everything')

