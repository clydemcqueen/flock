#!/usr/bin/env python

# TODO: 
# listen to takeoff, land and turn on/off cmd_vel publishing appropriately ./
# better controller for movement (move faster when farther away, slower when closer) use PID ./
# remember last turn direction, so will turn that way when 'turning to find human' ./
# add support for no debug aka just look at objects and don't do any image manip ./
# make sure only one human is spotted at a time

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray
import message_filters

import cv2
import numpy as np
import sys
from time import sleep
from pid import PID # probs bloat for our use case, could get away with just [kp, error]

DO_DEBUG=True
#
OBJECTS_TOPIC_SUB = 'objects'
HUMAN_ID = 1
IMAGE_TOPIC_SUB = 'debug_image'
DEBUG_IMAGE_TOPIC_PUB = 'debug_image_centroid'
#
WIDTH_KEEPING_FOV  = 200 # pixels
HEIGHT_KEEPING_FOV = 100 # pixels
DISTANCE_KEEP_AWAY = 600 # unit ? (from Detection2DArray) 
DISTANCE_KEEPING_RANGE = 30 # unit ? (from Detection2DArray) 
KP = 0.0025 # u/error i.e. 0.5/200
SLOW_TURN = 0.2
FAST_TURN = 0.5

class FollowHuman:
    def __init__(self):
	rospy.loginfo('FollowHuman class constructor')

	# Image
	self.cv_bridge = CvBridge()
	if DO_DEBUG:
	    self.image_sub = message_filters.Subscriber(IMAGE_TOPIC_SUB, Image)
	    self.objects_sub = message_filters.Subscriber(OBJECTS_TOPIC_SUB, Detection2DArray)
	    ts = message_filters.TimeSynchronizer([self.image_sub, self.objects_sub], 10)
	    ts.registerCallback(self.image_and_object_cb)

	    self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC_PUB, Image, queue_size=10)
	else:
	    self.objects_sub = rospy.Subscriber(OBJECTS_TOPIC_SUB, Detection2DArray, self.objects_cb, queue_size=1)

	# Drone
	self._takeoff_sub = rospy.Subscriber('takeoff', Empty, self.takeoff_cb, queue_size=10)
	self._land_sub = rospy.Subscriber('land', Empty, self.land_cb, queue_size=10)
	self.is_commandable = False # only publish commands when airborne
	#
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	self.cmd = '' 
	#
	self.human_last_seen_dir = 'left'

    def takeoff_cb(self):
	sleep(0.5)
	self.is_commandable = True
    def land_cb(self):
	sleep(0.5)
	self.is_commandable = False 


    def image_and_object_cb(self, imgmsg, objects): # used when debugging
	if not self.is_commandable:
	    return      

	try: 
            img = self.cv_bridge.imgmsg_to_cv2(imgmsg)
	except CvBridgeError:
	    rospy.logwarn('CvBridge could not convert to cv2')
	    return
	self.img_height, self.img_width, _ = img.shape

	#print(len(objects.detections))

	human = self.get_human_obj(objects)
	if human is None:
	    self.find_human()

	else:
	    cx, cy, bound_width = self.get_object_attrs(human)

	    # show center
	    cv2.rectangle(img,(int(cx-5),int(cy+5)),(int(cx+5),int(cy-5)),(255,255,255),1)
	    # show bound size
	    cv2.putText(img, 'bound_width: '+str(bound_width), (40,660), 
		cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2,cv2.LINE_AA)

	    self.follow_human(cx, cy, bound_width)	

	# show bounds
	cv2.rectangle(img, 
		(self.img_width/2-WIDTH_KEEPING_FOV, self.img_height/2+HEIGHT_KEEPING_FOV),	
		(self.img_width/2+WIDTH_KEEPING_FOV, self.img_height/2-HEIGHT_KEEPING_FOV),
		(255,255,255),3)	
	cv2.putText(img,self.cmd,(40,60), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2,cv2.LINE_AA)

	try:
	    imgmsg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
	    self.debug_image_pub.publish(imgmsg)
	except:
	    rospy.logwarn('CvBridge could not convert to imgmsg')
	    return
	
    def objects_cb(self, objects):
	if not self.is_commandable:
	    return      

	human = self.get_human_obj(objects)
	if human is None:
	    self.find_human()
	else:
            cx, cy, bound_width = self.get_object_attrs(human)
	    self.follow_human(cx, cy, bound_width)	

    def get_human_obj(self, objects):
	for obj in objects.detections:
	    if obj.results[0].id == HUMAN_ID:
	    	return obj
	return None 
    def get_object_attrs(self, obj):
	cx, cy = (obj.bbox.center.x*4/3, obj.bbox.center.y*3/4) #weird scaling
	#print cx, cy
	bound_width = obj.bbox.size_y		    
	return cx, cy, bound_width

	
    def follow_human(self, cx, cy, bound_width):
	lx = lz = az = 0

	# move yaw to center human
	if cx < (self.img_width/2 - WIDTH_KEEPING_FOV):
	    az = SLOW_TURN
	    self.cmd = 'turn ccw'
	    self.human_last_seen_dir = 'left'
	elif cx > (self.img_width/2 + WIDTH_KEEPING_FOV):
	    az = -SLOW_TURN
	    self.cmd = 'turn cw'
	    self.human_last_seen_dir = 'right'
	else: # locked on, move towards or away from human
	    if bound_width < (DISTANCE_KEEP_AWAY-DISTANCE_KEEPING_RANGE):
		error = abs(bound_width - (DISTANCE_KEEP_AWAY-DISTANCE_KEEPING_RANGE))
		ucontrol = KP * error	
		lx = ucontrol
	        self.cmd = 'move forward '+str(ucontrol)
	    elif bound_width > (DISTANCE_KEEP_AWAY+DISTANCE_KEEPING_RANGE):
		error = abs(bound_width - (DISTANCE_KEEP_AWAY+DISTANCE_KEEPING_RANGE))
		ucontrol = self.KP * error	
		lx = -ucontrol
	        self.cmd = 'move backward '+str(ucontrol)
	    else:
	        self.cmd = 'do nothing'
	
	self.move(lx,lz,az) 
 
    def find_human(self):
	if self.human_last_seen_dir = 'left':
	    self.cmd = 'turn left to find human'
            self.move(0,0,FAST_TURN) 
	else:
	    self.cmd = 'turn right to find human'
            self.move(0,0,-FAST_TURN) 
	


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

