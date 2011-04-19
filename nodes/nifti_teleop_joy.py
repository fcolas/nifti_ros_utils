#!/usr/bin/env python
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from geometry_msgs.msg import Twist
from joy.msg import Joy
from nifti_ros_drivers.msg import FlippersState
from math import pi


class NiftiTeleopJoy(object):

	def __init__(self):
		# flag to stop the robot is deadman is released
		self.sending = False
		# joystick axes
		self.lin_vel_axis = rospy.get_param('~axis_linear', 1)
		self.ang_vel_axis = rospy.get_param('~axis_angular', 0)
		# joystick buttons
		self.deadman_button = rospy.get_param('~deadman_button', 1)
		self.run_button = rospy.get_param('~run_button', 0)
		self.flipper_buttons = [
				rospy.get_param('~front_left_flipper_button', 4),
				rospy.get_param('~rear_left_flipper_button', 6),
				rospy.get_param('~front_right_flipper_button', 5),
				rospy.get_param('~rear_right_flipper_button', 7)]
		self.flipper_axis = rospy.get_param('~flipper_axis', 5)
		# speed limits
		self.max_lin_vel = rospy.get_param('~max_linear', 0.3)
		self.max_ang_vel = rospy.get_param('~max_angular', 0.8)
		self.max_lin_vel_run = rospy.get_param('~max_linear_run', 0.55)
		self.max_ang_vel_run = rospy.get_param('~max_angular_run', 2.75)
		self.flipper_increment = rospy.get_param('~flipper_increment', 5*pi/180.)
		# publisher and subscribers
		command_topic = rospy.get_param('~teleop_cmd_vel', '/cmd_vel')
		self.cmdvel_pub = rospy.Publisher(command_topic, Twist)
		flipper_topic = rospy.get_param('~teleop_flippers_cmd',
				'/flippers_cmd')
		self.flippers_pub = rospy.Publisher(flipper_topic, FlippersState)
		rospy.Subscriber('joy', Joy, self.joyCallBack)
		self.flippers_ok = False
		rospy.Subscriber('flippers_state', FlippersState, self.flippersCallBack)


	def flippersCallBack(self, flippers):
		self.flippers = [flippers.a0, flippers.a1, flippers.a2, flippers.a3]
		self.flippers_ok = True
			

	def joyCallBack(self, joy):
		self.cmdvel_cb(joy)
		self.flipper_cb(joy)


	def cmdvel_cb(self, joy):
		tw = Twist()
		if joy.buttons[self.deadman_button]:
			self.sending = True
			if joy.buttons[self.run_button]:
				lin_scale = self.max_lin_vel_run
				ang_scale = self.max_ang_vel_run
			else:
				lin_scale = self.max_lin_vel
				ang_scale = self.max_ang_vel
			tw.linear.x = lin_scale*joy.axes[self.lin_vel_axis]
			tw.angular.z = ang_scale*joy.axes[self.ang_vel_axis]
			self.cmdvel_pub.publish(tw)

		elif self.sending:	# make sure we ask the robot to stop
			tw.linear.x = 0.0
			tw.angular.z = 0.0
			self.cmdvel_pub.publish(tw)
			self.sending = False
		#else:
		#	pass	# we don't publish constantly

			
	def flipper_cb(self, joy):
		if joy.buttons[self.deadman_button]\
				and any([joy.buttons[b] for b in self.flipper_buttons])\
				and joy.axes[self.flipper_axis]:
			if not self.flippers_ok:
				rospy.logwarn('Flipper command disabled since no FlippersState message received.')
			fs = FlippersState()
			for i, b in enumerate(self.flipper_buttons):
				a = self.flippers[i]
				if joy.buttons[b]:
					a += joy.axes[self.flipper_axis]*self.flipper_increment
				setattr(fs, 'a%d'%i, a)
			self.flippers_pub.publish(fs)
		#else:
		#	pass	# we don't publish constantly




def main():
	try:
		rospy.init_node('nifti_teleop_joy')
		ntj = NiftiTeleopJoy()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
		
