#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import tf
import tf.transformations
import numpy as np


class DetectArUco(object):

    # Clyde's Tello calibration data
    _camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
    _distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

    # Markers are 18cm x 18cm
    _marker_length = 0.18

    def __init__(self):
        # Initialize ROS
        rospy.init_node('detect_aruco_node', anonymous=False)

        # ROS publishers
        self._image_pub = rospy.Publisher('image_marked', Image, queue_size=10)
        self._tf_broadcaster = tf.TransformBroadcaster()

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

        # Compute pose
        rvecs, tvecs, object_points = cv2.aruco.estimatePoseSingleMarkers(corners, self._marker_length, self._camera_matrix, self._distortion)

        # Publish pose to marker0 as a TF message
        if tvecs is not None and rvecs is not None:
            # Rodrigues => 3x3 matrix
            R3x3, _ = cv2.Rodrigues(rvecs[0][0])

            # transformations.py wants a 4x4 matrix -- probably a better way to do this
            R4x4 = np.zeros((4, 4))
            R4x4[:3, 0] = R3x3[:, 0]
            R4x4[:3, 1] = R3x3[:, 1]
            R4x4[:3, 2] = R3x3[:, 2]
            R4x4[3,3] = 1

            # 4x4 matrix => quaternion
            q = tf.transformations.quaternion_from_matrix(R4x4)

            # Broadcast transform
            self._tf_broadcaster.sendTransform(tvecs[0][0], q, rospy.Time.now(), 'marker_frame', 'camera_frame')

        # Draw border on the color image
        color_mat = cv2.aruco.drawDetectedMarkers(color_mat, corners)

        # Debugging display
        # cv2.imshow('Markers', color_mat)
        # cv2.waitKey(1)

        # Publish marked up image
        self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))


if __name__ == '__main__':
    driver = DetectArUco()
