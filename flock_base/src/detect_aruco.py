#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class DetectArUco(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('detect_aruco_node', anonymous=True)

        # ROS publishers
        self._image_pub = rospy.Publisher('image_marked', Image, queue_size=10)

        # ROS subscriptions
        rospy.Subscriber("image_raw", Image, self.image_callback)

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        # ArUco data -- we're using 6x6 ArUco images
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self._aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Spin until interrupted
        rospy.spin()

    def image_callback(self, msg):
        # Convert ROS image ==> OpenCV Mat
        color_mat = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Color => gray for detection
        gray_mat = cv2.cvtColor(color_mat, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray_mat, self._aruco_dict, parameters=self._aruco_parameters)

        # Publish list of corners
        # TODO

        # Draw border on the color image
        color_mat = cv2.aruco.drawDetectedMarkers(color_mat, corners)

        # Debugging display TODO remove this
        cv2.imshow('Markers', color_mat)
        cv2.waitKey(1)

        # Publish marked up image
        self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))


if __name__ == '__main__':
    driver = DetectArUco()
