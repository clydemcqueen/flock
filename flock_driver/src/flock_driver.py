#!/usr/bin/env python

import threading
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from flock_msgs.msg import Flip, FlightData
import cv2
import numpy as np
import tellopy
import socket
from cv_bridge import CvBridge

# Use PyAV - many will have trouble installing so might not use
USE_PyAV=False
if USE_PyAV:
    import av



class FlockDriver(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('flock_driver_node', anonymous=False)

        # ROS publishers
        self._flight_data_pub = rospy.Publisher('flight_data', FlightData, queue_size=10)
        self._image_pub = rospy.Publisher('image_raw', Image, queue_size=10)

        # ROS subscriptions
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber('land', Empty, self.land_callback)
        rospy.Subscriber('flip', Flip, self.flip_callback)

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        # Connect to the drone
        self._drone = tellopy.Tello()
        self._drone.connect()
        self._drone.wait_for_connection(60.0)

        # Start video thread
	if USE_PyAV:
            self._stop_request = threading.Event()
            video_thread = threading.Thread(target=self.video_worker)
            video_thread.start()

	else:
    	    self.loopback = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

	    self._drone.start_video()
	    self._drone.subscribe(self._drone.EVENT_VIDEO_FRAME, self.videoFrameHandler)

            self._stop_request = threading.Event()
	    video_thread = threading.Thread(None, self.cam) # Start thread
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

    def flight_data_callback(self, event, sender, data, **args):
        flight_data = FlightData()

        # Battery state
        flight_data.battery_percent = data.battery_percentage
        flight_data.estimated_flight_time_remaining = data.drone_fly_time_left / 10.

        # Flight mode
        flight_data.flight_mode = data.fly_mode

        # Flight time
        flight_data.flight_time = data.fly_time

        # Very coarse velocity data
        # TODO do east and north refer to the body frame?
        # TODO the / 10. conversion might be wrong, verify
        flight_data.east_speed = -1. if data.east_speed > 30000 else data.east_speed / 10.
        flight_data.north_speed = -1. if data.north_speed > 30000 else data.north_speed / 10.
        flight_data.ground_speed = -1. if data.ground_speed > 30000 else data.ground_speed / 10.

        # Altitude
        flight_data.altitude = -1. if data.height > 30000 else data.height / 10.

        # Equipment status
        flight_data.equipment = data.electrical_machinery_state
        flight_data.high_temperature = data.temperature_height

        # Some state indicators?
        flight_data.em_ground = data.em_ground
        flight_data.em_sky = data.em_sky
        flight_data.em_open = data.em_open

        # Publish what we have
        self._flight_data_pub.publish(flight_data)

        # Debugging: is there data here? Print nonzero values
        if data.battery_low:
            print('battery_low is nonzero: %d' % data.battery_low)
        if data.battery_lower:
            print('battery_lower is nonzero: %d' % data.battery_lower)
        if data.battery_state:
            print('battery_state is nonzero: %d' % data.battery_state)
        if data.drone_battery_left:
            print('drone_battery_left is nonzero: %d' % data.drone_battery_left)
        if data.camera_state:
            print('camera_state is nonzero: %d' % data.camera_state)
        if data.down_visual_state:
            print('down_visual_state is nonzero: %d' % data.down_visual_state)
        if data.drone_hover:
            print('drone_hover is nonzero: %d' % data.drone_hover)
        if data.factory_mode:
            print('factory_mode is nonzero: %d' % data.factory_mode)
        if data.front_in:
            print('front_in is nonzero: %d' % data.front_in)
        if data.front_lsc:
            print('front_lsc is nonzero: %d' % data.front_lsc)
        if data.front_out:
            print('front_out is nonzero: %d' % data.front_out)
        if data.gravity_state:
            print('gravity_state is nonzero: %d' % data.gravity_state)
        if data.imu_calibration_state:
            print('imu_calibration_state is nonzero: %d' % data.imu_calibration_state)
        if data.imu_state:
            print('imu_state is nonzero: %d' % data.imu_state)
        if data.outage_recording:
            print('outage_recording is nonzero: %d' % data.outage_recording)
        if data.power_state:
            print('power_state is nonzero: %d' % data.power_state)
        if data.pressure_state:
            print('pressure_state is nonzero: %d' % data.pressure_state)
        if data.throw_fly_timer:
            print('throw_fly_timer is nonzero: %d' % data.throw_fly_timer)
        if data.wind_state:
            print('wind_state is nonzero: %d' % data.wind_state)

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


    # for using PyAV    
    def video_worker(self):

        # Get video stream, open in PyAV
        container = av.open(self._drone.get_video_stream())

        # Decode h264
        rospy.loginfo('starting video pipeline')
        for frame in container.decode(video=0):

            # Convert PyAV frame => PIL image => OpenCV Mat
            color_mat = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)

            # Convert OpenCV Mat => ROS Image message and publish
            self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

            # Check for normal shutdown
            if self._stop_request.isSet():
                return


    def cam(self): # RUN THE WHILE LOOP AS FAST AS POSSIBLE!
    ##           VIDEO DISTORTION OTHERWISE! 
        try:
            cap = cv2.VideoCapture("udp://@127.0.0.1:5000") # Random address
            if not cap.isOpened:
                cap.open()

            while not self._stop_request.isSet():
                res, frame = cap.read()
		self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))

        except Exception as e:
	    print(e)
        finally:
            cap.release()
            print("Video Stream stopped.")

    def videoFrameHandler(self, event, sender, data):
	self.loopback.sendto(data, ('127.0.0.1', 5000)) # random address


if __name__ == '__main__':
    driver = FlockDriver()



