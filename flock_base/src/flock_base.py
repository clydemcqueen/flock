#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from flock_msgs.msg import Flip

# XBox One joystick axes and buttons
_joy_axis_left_lr = 0           # Left stick left/right; 1.0 is left and -1.0 is right
_joy_axis_left_fb = 1           # Left stick forward/back; 1.0 is forward and -1.0 is back
_joy_axis_left_trigger = 2      # Left trigger
_joy_axis_right_lr = 3          # Right stick left/right; 1.0 is left and -1.0 is right
_joy_axis_right_fb = 4          # Right stick forward/back; 1.0 is forward and -1.0 is back
_joy_axis_right_trigger = 5     # Right trigger
_joy_axis_trim_lr = 6           # Trim left/right; 1.0 for left and -1.0 for right
_joy_axis_trim_fb = 7           # Trim forward/back; 1.0 for forward and -1.0 for back
_joy_button_A = 0               # A button
_joy_button_B = 1               # B button
_joy_button_X = 2               # X button
_joy_button_Y = 3               # Y button
_joy_button_left_bumper = 4     # Left bumper
_joy_button_right_bumper = 5    # Right bumper
_joy_button_view = 6            # View button
_joy_button_menu = 7            # Menu button
_joy_button_shift = 8           # XBox logo button
_joy_button_left_stick = 9      # Left stick button
_joy_button_right_stick = 10    # Right stick button


class FlockBase(object):

    def __init__(self):
        rospy.init_node('flock_base_node', anonymous=False)

        # Joystick assignments
        left_handed = rospy.get_param('~left_handed', False)    # ~ means private, e.g., /flock_base_node/left_handed
        self.joy_axis_throttle = _joy_axis_left_fb if left_handed else _joy_axis_right_fb
        self.joy_axis_strafe = _joy_axis_right_lr
        self.joy_axis_vertical = _joy_axis_right_fb if left_handed else _joy_axis_left_fb
        self.joy_axis_yaw = _joy_axis_left_lr
        self.joy_button_takeoff = _joy_button_menu
        self.joy_button_land = _joy_button_view
        self.joy_button_flip_forward = _joy_button_Y
        self.joy_button_flip_left = _joy_button_X
        self.joy_button_flip_right = _joy_button_B
        self.joy_button_flip_back = _joy_button_A
        self.joy_button_left_bumper = _joy_button_left_bumper
        self.joy_axis_trim_lr = _joy_axis_trim_lr
        self.joy_axis_trim_fb = _joy_axis_trim_fb

        # Trim axis state
        self.joy_axis_trim_lr_pressed = False
        self.joy_axis_trim_fb_pressed = False

        # speed for trim commands
        self.speed = 100

        # Publications
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._takeoff_pub = rospy.Publisher('takeoff', Empty, queue_size=10)
        self._land_pub = rospy.Publisher('land', Empty, queue_size=10)
        self._flip_pub = rospy.Publisher('flip', Flip, queue_size=10)

        # Subscriptions
        rospy.Subscriber('joy', Joy, self.joy_callback)

        # Spin until interrupted
        rospy.spin()

    def joy_callback(self, msg):
        left_bumper_pressed = msg.buttons[self.joy_button_left_bumper] != 0

        joy_axis_trim_lr_state = msg.axes[self.joy_axis_trim_lr]
        flip_command = None
        if joy_axis_trim_lr_state > 0:
            flip_command = Flip.move_yawleft if left_bumper_pressed else Flip.move_left
            self.joy_axis_trim_lr_pressed = True
        elif joy_axis_trim_lr_state < 0:
            flip_command = Flip.move_yawright if left_bumper_pressed else Flip.move_right
            self.joy_axis_trim_lr_pressed = True
        elif self.joy_axis_trim_lr_pressed:
            flip_command = Flip.move_yaw_none if left_bumper_pressed else Flip.move_lr_none
            self.joy_axis_trim_lr_pressed = False
        if flip_command:
            self._flip_pub.publish(Flip(flip_command=flip_command))

        joy_axis_trim_fb_state = msg.axes[self.joy_axis_trim_fb]
        flip_command = None
        if joy_axis_trim_fb_state > 0:
            flip_command = Flip.move_up if left_bumper_pressed else Flip.move_forward
            self.joy_axis_trim_fb_pressed = True
        elif joy_axis_trim_fb_state < 0:
            flip_command = Flip.move_down if left_bumper_pressed else Flip.move_back
            self.joy_axis_trim_fb_pressed = True
        elif self.joy_axis_trim_fb_pressed:
            flip_command = Flip.move_ud_none if left_bumper_pressed else Flip.move_fb_none
            self.joy_axis_trim_fb_pressed = False
        if flip_command:
            self._flip_pub.publish(Flip(flip_command=flip_command))

        if not self.joy_axis_trim_lr_pressed and not self.joy_axis_trim_fb_pressed:
            twist = Twist()
            twist.linear.x = msg.axes[self.joy_axis_throttle]   # ROS body frame convention: +x is forward, -x is back
            twist.linear.y = msg.axes[self.joy_axis_strafe]     # ROS body frame convention: +y is left, -y is right
            twist.linear.z = msg.axes[self.joy_axis_vertical]   # ROS body frame convention: +z is ascend, -z is descend
            twist.angular.z = msg.axes[self.joy_axis_yaw]       # ROS body frame convention: +yaw is ccw, -yaw is cw
            self._cmd_vel_pub.publish(twist)

        if msg.buttons[self.joy_button_takeoff] != 0:
            self._takeoff_pub.publish()
        elif msg.buttons[self.joy_button_land] != 0:
            self._land_pub.publish()

        if msg.buttons[self.joy_button_flip_forward] != 0:
            self._flip_pub.publish(Flip(flip_command=Flip.flip_forward))
        elif msg.buttons[self.joy_button_flip_left] != 0:
            self._flip_pub.publish(Flip(flip_command=Flip.flip_left))
        elif msg.buttons[self.joy_button_flip_right] != 0:
            self._flip_pub.publish(Flip(flip_command=Flip.flip_right))
        elif msg.buttons[self.joy_button_flip_back] != 0:
            self._flip_pub.publish(Flip(flip_command=Flip.flip_back))


if __name__ == '__main__':
    base = FlockBase()
