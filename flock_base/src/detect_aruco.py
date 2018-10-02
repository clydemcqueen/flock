#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import tf
import tf.transformations
import numpy as np

# Transformation notation:
# Tst == T_source_target
# vector_target = T_source_target * vector_source


def rvec_and_tvec_to_matrix(rvec, tvec):
    """Rodrigues rotation and translation vector to 4x4 matrix"""
    R, _ = cv2.Rodrigues(rvec)
    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


def tf_to_matrix(ros_transform):
    """ROS transform to 4x4 matrix"""
    t, q = ros_transform
    T = tf.transformations.quaternion_matrix(q)
    T[:3, 3] = t
    return T


def matrix_to_tf(T):
    """4x4 matrix to ROS transform"""
    t = T[:3, 3]
    q = tf.transformations.quaternion_from_matrix(T)
    return (t, q)


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
        time = rospy.Time()
        wait = rospy.Duration(4)
        self._tf_listener.waitForTransform("marker_frame", "odom", time, wait)  # target, source
        self._Tom = tf_to_matrix(self._tf_listener.lookupTransform("marker_frame", "odom", time))  # target, source
        self._tf_listener.waitForTransform("base_link", "camera_frame", time, wait)
        self._Tcb = tf_to_matrix(self._tf_listener.lookupTransform("base_link", "camera_frame", time))

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
        # color_mat = cv2.aruco.drawDetectedMarkers(color_mat, corners)

        # Publish the marked up image
        # self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

        # Compute transformations, each is marker_frame => camera_frame
        rvecs, tvecs, object_points = cv2.aruco.estimatePoseSingleMarkers(corners, self._marker_length, self._camera_matrix, self._distortion)

        for index in range(len(ids)):
            if ids[index][0] == self._first_marker:
                # Tob = Tcb * Tmc * Tom
                Tmc = rvec_and_tvec_to_matrix(rvecs[index][0], tvecs[index][0])
                Tob = self._Tcb.dot(Tmc).dot(self._Tom)

                # Broadcast the tf transform odom => base_link, where child=target, parent=source
                # We can't flip source and target because we want odom to be the parent of base_link
                # Instead, we need to invert matrix Tob to get Tbo
                Tbo = tf.transformations.inverse_matrix(Tob)
                t, q = matrix_to_tf(Tbo)
                self._tf_broadcaster.sendTransform(t, q, rospy.Time.now(), child='base_link', parent='odom')
                break

        for index in range(len(ids)):
            # TODO compute pose of marker in odom frame
            # TODO draw marker in rviz
            pass


if __name__ == '__main__':
    driver = DetectArUco()
