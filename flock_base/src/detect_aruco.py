#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import tf
import tf.transformations
import numpy as np


def rodrigues_to_quaternion(R):
    # Rodrigues => 3x3 matrix
    R3x3, _ = cv2.Rodrigues(R)

    # transformations.py wants a 4x4 matrix -- probably a better way to do this
    R4x4 = np.zeros((4, 4))
    R4x4[:3, 0] = R3x3[:, 0]
    R4x4[:3, 1] = R3x3[:, 1]
    R4x4[:3, 2] = R3x3[:, 2]
    R4x4[3,3] = 1

    # 4x4 matrix => quaternion
    return tf.transformations.quaternion_from_matrix(R4x4)


class DetectArUco(object):

    # Clyde's Tello calibration data
    _camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
    _distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

    # Markers are 18cm x 18cm
    _marker_length = 0.18

    # ID of the first marker we saw
    _first_marker = -1

    def __init__(self):
        # ArUco data -- we're using 6x6 ArUco images
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self._aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Initialize ROS
        rospy.init_node('detect_aruco_node', anonymous=False)

        # ROS publishers
        self._image_pub = rospy.Publisher('image_marked', Image, queue_size=10)

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        # ROS transform managers
        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()

        # Get a few key static transforms
        self._tf_listener.waitForTransform("odom", "first_marker_frame", rospy.Time(), rospy.Duration(4))
        self._odom_to_first_marker_frame = self._tf_listener.lookupTransform("odom", "first_marker_frame", rospy.Time())
        print(self._odom_to_first_marker_frame)
        self._tf_listener.waitForTransform("base_link", "camera_frame", rospy.Time(), rospy.Duration(4))
        self._base_link_to_camera_frame = self._tf_listener.lookupTransform("base_link", "camera_frame", rospy.Time())
        print(self._base_link_to_camera_frame)

        # Now that we're initialized, set up subscriptions and spin
        rospy.Subscriber("image_raw", Image, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        # Convert ROS image ==> OpenCV Mat
        color_mat = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Color => gray for detection
        gray_mat = cv2.cvtColor(color_mat, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray_mat, self._aruco_dict, parameters=self._aruco_parameters)

        # Stop if no markers were detected
        if ids is None:
            return

        # Grab the first marker we see
        if self._first_marker < 0:
            self._first_marker = ids[0][0]
            rospy.loginfo('First marker has id %d' % self._first_marker)

        # Draw borders on the color image
        color_mat = cv2.aruco.drawDetectedMarkers(color_mat, corners)

        # Publish the marked up image
        self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

        # Compute transformations, each is marker_frame => camera_frame
        rvecs, tvecs, object_points = cv2.aruco.estimatePoseSingleMarkers(corners, self._marker_length, self._camera_matrix, self._distortion)

        for index in range(len(ids)):
            if ids[index][0] == self._first_marker:
                # Compute odom => base_link from these transformations:
                #     self._odom_to_marker_frame
                #     self._base_link_to_camera_frame
                #     camera_frame => marker_frame
                # TODO

                # Broadcast camera_frame => marker_frame
                # TODO broadcast the odom => base_link transform instead
                T = tvecs[index][0]
                q = rodrigues_to_quaternion(rvecs[index][0])
                self._tf_broadcaster.sendTransform(T, q, rospy.Time.now(), child='marker_frame', parent='camera_frame')
                break

        for index in range(len(ids)):
            # TODO compute pose of marker in odom frame
            # TODO draw marker in rviz
            pass


if __name__ == '__main__':
    driver = DetectArUco()
