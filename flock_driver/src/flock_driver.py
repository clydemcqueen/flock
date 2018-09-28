#!/usr/bin/env python

import threading
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from flock_msgs.msg import Flip
import av
import cv2
import numpy
import tellopy


class FlockDriver(object):

    def __init__(self):
        # Connect to the drone
        self._drone = tellopy.Tello()
        self._drone.connect()
        self._drone.wait_for_connection(60.0)
        rospy.loginfo('connected to drone')

        # Initialize ROS
        rospy.init_node('flock_driver_node', anonymous=True)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber('land', Empty, self.land_callback)
        rospy.Subscriber('flip', Flip, self.flip_callback)

        # Start video thread
        self._stop_request = threading.Event()
        video_thread = threading.Thread(target=self.video_worker)
        video_thread.start()

        # Spin until interrupted
        rospy.spin()

        # Force a landing
        self._drone.land()

        # Stop the video thread
        self._stop_request.set()
        video_thread.join(timeout=2)

        # Shut down the drone
        self._drone.quit()
        self._drone = None

    def cmd_vel_callback(self, msg):
        self._drone.set_pitch(msg.linear.x)
        self._drone.set_roll(-msg.linear.y)     # Note sign flip
        self._drone.set_throttle(msg.linear.z)
        self._drone.set_yaw(-msg.angular.z)     # Note sign flip

    def takeoff_callback(self, msg):
        self._drone.takeoff()

    def land_callback(self, msg):
        self._drone.land()

    def flip_callback(self, msg):
        if msg.flip_command == Flip.flip_forward:
            self._drone.flip_forward()
        elif msg.flip_command == Flip.flip_back:
            self._drone.flip_back()
        elif msg.flip_command == Flip.flip_left:
            self._drone.flip_left()
        elif msg.flip_command == Flip.flip_right:
            self._drone.flip_right()
        elif msg.flip_command == Flip.flip_forwardleft:
            self._drone.flip_forwardleft()
        elif msg.flip_command == Flip.flip_forwardright:
            self._drone.flip_forwardright()
        elif msg.flip_command == Flip.flip_backleft:
            self._drone.flip_backleft()
        elif msg.flip_command == Flip.flip_backright:
            self._drone.flip_backright()

    def video_worker(self):
        # Design choice (for now): process images here, vs. sending decompressed video images to another node
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()

        # Get video stream, open in PyAV
        container = av.open(self._drone.get_video_stream())

        # Decode h264
        rospy.loginfo('starting video pipeline')
        for frame in container.decode(video=0):

            # Convert PyAV frame => PIL image => OpenCV Mat
            color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

            # Color => gray for detection
            gray_mat = cv2.cvtColor(color_mat, cv2.COLOR_BGR2GRAY)

            # Detect markers
            corners, ids, _ = cv2.aruco.detectMarkers(gray_mat, aruco_dict, parameters=parameters)

            # Publish list of corners
            # TODO

            # Draw border on the color image
            color_mat = cv2.aruco.drawDetectedMarkers(color_mat, corners)
            cv2.imshow('Markers', color_mat)
            cv2.waitKey(1)

            # Check for normal shutdown
            if self._stop_request.isSet():
                return


if __name__ == '__main__':
    driver = FlockDriver()
