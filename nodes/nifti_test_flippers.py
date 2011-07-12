#!/usr/bin/env python

import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from joy.msg import Joy

from nifti_robot_driver_msgs.msg import FlippersState, RobotStatus, FlipperCommand

from math import pi

from nifti_teleop_joy import HistoryJoystick


ID_FLIPPER_FRONT_LEFT=3
ID_FLIPPER_FRONT_RIGHT=4
ID_FLIPPER_REAR_LEFT=5
ID_FLIPPER_REAR_RIGHT=6


class NiftiTestFlippers(object):

	def __init__(self):
		self.joy = HistoryJoystick()
		self.deadman_button = 1
		self.rl_button = 6
		self.reset_button = 3
		self.flipper_axis = 5
		self.flipper_pub = rospy.Publisher('/flipper_cmd', FlipperCommand)
		self.enable_pub = rospy.Publisher('/enable', Bool)
		rospy.Subscriber('/joy', Joy, self.joyCallBack)

	def joyCallBack(self, joy):
		self.joy.update(joy)
		self.deadman_jcb(self.joy)
		self.flipper_jcb(self.joy)


	def deadman_jcb(self, joy):
		if joy.released(self.deadman_button):
			self.enable_pub.publish(False)
		elif joy.pressed(self.deadman_button):
			self.enable_pub.publish(True)


	def flipper_jcb(self, joy):
		if joy.buttons[self.deadman_button]:
			flipper_command = FlipperCommand()
			if joy.buttons[self.rl_button]:
				flipper_command.object_id = ID_FLIPPER_REAR_LEFT
				if joy.pressed(self.reset_button):
					flipper_command.angle = 0.
					self.flipper_pub.publish(flipper_command)
				if joy.axis_touched(self.flipper_axis):
					flipper_command.angle = joy.axes[self.flipper_axis]*pi/2.
					self.flipper_pub.publish(flipper_command)

def main():
	try:
		rospy.init_node('nifti_test_flippers')
		ntf = NiftiTestFlippers()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__=='__main__':
	main()
