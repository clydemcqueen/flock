#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from flock_msgs.msg import Flip

# XBox One joystick
_joy_axis_yaw_ = 0              # Left stick left/right; 1.0 is left and -1.0 is right
_joy_axis_forward = 1           # Left stick up/down; 1.0 is forward and -1.0 is backward
_joy_axis_strafe = 3            # Right stick left/right; 1.0 is left and -1.0 is right
_joy_axis_vertical = 4          # Right stick up/down; 1.0 is ascend and -1.0 is descend
_joy_axis_trim_lr = 6           # Trim left/right; acts like 2 buttons; 1.0 for left and -1.0 for right -- unassigned
_joy_axis_trim_fb = 7           # Trim up/down; acts like 2 buttons; 1.0 for fwd and -1.0 for back -- unassigned
_joy_axis_left_trigger = 8      # Left trigger -- unassigned
_joy_axis_right_trigger = 9     # Right trigger -- unassigned
_joy_button_view = 6            # View button -- land
_joy_button_menu = 7            # Menu button -- takeoff
_joy_button_A = 0               # A button -- flip forward
_joy_button_X = 2               # X button -- flip left
_joy_button_B = 1               # B button -- flip right
_joy_button_Y = 3               # Y button -- flip forward
_joy_button_left_bumper = 4     # Left bumper -- unassigned
_joy_button_right_bumper = 5    # Right bumper --unassigned
_joy_button_left_stick = 9      # Left stick button -- unassigned
_joy_button_right_stick = 10    # Right stick button -- unassigned


class FlockBase(object):

    def __init__(self):
        rospy.init_node('flock_base', anonymous=True)
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._takeoff_pub = rospy.Publisher('takeoff', Empty, queue_size=10)
        self._land_pub = rospy.Publisher('land', Empty, queue_size=10)
        self._flip_pub = rospy.Publisher('flip', Flip, queue_size=10)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.spin()

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[_joy_axis_forward]
        twist.linear.y = -msg.axes[_joy_axis_strafe]
        twist.angular.z = -msg.axes[_joy_axis_yaw_]
        twist.linear.z = msg.axes[_joy_axis_vertical]
        self._cmd_vel_pub.publish(twist)

        if msg.buttons[_joy_button_menu] != 0:
            self._takeoff_pub.publish()
        elif msg.buttons[_joy_button_view] != 0:
            self._land_pub.publish()

        if msg.buttons[_joy_button_Y] != 0:
            self._flip_pub.publish(Flip(flip_command=Flip.flip_forward))
        elif msg.buttons[_joy_button_X] != 0:
            self._flip_pub.publish(Flip(flip_command=Flip.flip_left))
        elif msg.buttons[_joy_button_B] != 0:
            self._flip_pub.publish(Flip(flip_command=Flip.flip_right))
        elif msg.buttons[_joy_button_A] != 0:
            self._flip_pub.publish(Flip(flip_command=Flip.flip_back))


if __name__ == '__main__':
    base = FlockBase()
