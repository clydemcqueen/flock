#!/usr/bin/env python

# non-ros code taken from https://github.com/code-by-dt, with modifications

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
from cv_bridge import CvBridge, CvBridgeError

from keras import backend as K
from keras.models import load_model
from time import sleep
from keras.preprocessing.image import img_to_array
from keras.preprocessing import image
import cv2
import numpy as np
from time import sleep

import rospkg
rospack = rospkg.RosPack()
emotion_dir = rospack.get_path('flock_perception')+'/scripts/emotion_detection/'
#face_classifier = cv2.CascadeClassifier(emotion_dir+'haarcascade_frontalface_default.xml')
#classifier = load_model(emotion_dir+'Emotion_little_vgg.h5')


class_labels = ['Angry','Happy','Neutral','Sad','Surprise']

IMAGE_TOPIC_SUB = 'image_raw'
DEBUG_IMAGE_PUB = 'debug_image_emotion'
EMOTION_STATE_PUB = 'emotion_state'

class EmotionDetector:
    def __init__(self):
	rospy.loginfo('EmotionDetector class constructor')

	self.face_classifier = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
	self.classifier = load_model('Emotion_little_vgg.h5')

	self.counter = 0

	# Image
	self.cv_bridge = CvBridge()
	self.image_sub = rospy.Subscriber(IMAGE_TOPIC_SUB, Image, self.image_cb, queue_size=1)
	self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_PUB, Image, queue_size=1)
	self.emotion_pub = rospy.Publisher(EMOTION_STATE_PUB, String, queue_size=1)


    def image_cb(self, imgmsg):




	try: 
	    frame = self.cv_bridge.imgmsg_to_cv2(imgmsg)
	except CvBridgeError:
	    rospy.logwarn('CvBridge could not convert to cv2')
	    return

	# the meat
	labels = []
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	faces = self.face_classifier.detectMultiScale(gray,1.3,5)

	for (x,y,w,h) in faces:
	    cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
	    roi_gray = gray[y:y+h,x:x+w]
	    roi_gray = cv2.resize(roi_gray,(48,48),interpolation=cv2.INTER_AREA)
   	    # rect,face,image = face_detector(frame)

	    if np.sum([roi_gray])!=0:
	        roi = roi_gray.astype('float')/255.0
	        roi = img_to_array(roi)
	        roi = np.expand_dims(roi,axis=0)

		self.classifier._make_predict_function()
		self.graph = K.get_session().graph
		#self.graph = tf.get_default_graph()
		print('loaded classifier')


	        # make a prediction on the ROI, then lookup the class
		with self.graph.as_default():
	            preds = self.classifier.predict(roi)[0]
	            label=class_labels[preds.argmax()]
	            label_position = (x,y)
	            cv2.putText(frame,label,label_position,cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,0),3)
	    else:
	        cv2.putText(frame,'No Face Found',(20,60),cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,0),3)
	        label = 'NoFace'

	    self.emotion_pub.publish(label)
	    rospy.loginfo(label)

	try:
	    imgmsg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
	    self.debug_image_pub.publish(imgmsg)
	except:
	    rospy.logwarn('CvBridge could not convert to imgmsg')
	    return


if __name__ == "__main__":
    rospy.init_node('emotion_detector')
    e = EmotionDetector()
    rospy.spin()

