#!/usr/bin/env python
## @file
# Joystick teleoperation module for the NIFTi robot
#
# It handles velocity command of the platform, activation of the brake, motion
# of the flippers, and selection of the rolling speed of the laser scanner.
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from joy.msg import Joy
from nifti_ros_drivers.msg import FlippersState, RobotStatus
from math import pi

## Joystick teleoperation class
class NiftiTeleopJoy(object):
	'''Joystick teleoperation interface.

	It handles velocity command of the platform, activation of the brake, motion
	of the flippers, and selection of the rolling speed of the laser scanner.
	'''
	
	## Instantiate a NiftiTeleopJoy object
	def __init__(self):
		# joystick
		## proxy to enhance the Joy message with more functionalities (see
		# HistoryJoystick for details)
		self.joy = HistoryJoystick()
		# joystick axes
		## axis for linear velocity
		# @param ~axis_linear (default: 1)
		self.lin_vel_axis = rospy.get_param('~axis_linear', 1)
		## axis for angular velocity
		# @param ~axis_angular  (default: 0)
		self.ang_vel_axis = rospy.get_param('~axis_angular', 0)
		## axis for flippers command
		# @param ~flipper_axis (default: 5)
		self.flipper_axis = rospy.get_param('~flipper_axis', 5)
		## axis for scanning velocity
		# @param ~scanning_speed_axis (default: 4)
		self.scanning_speed_axis = rospy.get_param('~scanning_speed_axis', 4)
		# joystick buttons
		## deadman button
		# @param ~deadman_button (default: 1)
		self.deadman_button = rospy.get_param('~deadman_button', 1)
		## run button
		# @param ~run_button (default: 0)
		self.run_button = rospy.get_param('~run_button', 0)
		## button for the enable command
		# @param ~enable_button (default: 9)
		self.enable_button = rospy.get_param('~enable_button', 9)
		## button to toggle the differential brake
		# @param ~differential_brake_button (default: 8)
		self.brake_button = rospy.get_param('~differential_brake_button', 8)
		## button to select the front left flipper
		# @param ~deadman_button (default: 4)
		self.flipper_button_fl = rospy.get_param('~front_left_flipper_button', 4)
		## button to select the front right flipper
		# @param ~deadman_button (default: 5)
		self.flipper_button_fr = rospy.get_param('~front_right_flipper_button', 5)
		## button to select the rear left flipper
		# @param ~deadman_button (default: 6)
		self.flipper_button_rl = rospy.get_param('~rear_left_flipper_button', 6)
		## button to select the rear right flipper
		# @param ~deadman_button (default: 7)
		self.flipper_button_rr = rospy.get_param('~rear_right_flipper_button', 7)
		# speed limits
		## maximum linear velocity (in m/s)
		# @param /max_linear (default: 0.3)
		self.max_lin_vel = rospy.get_param('/max_linear', 0.3)
		## maximum angular velocity (in rad/s)
		# @param /max_angular (default: 0.8)
		self.max_ang_vel = rospy.get_param('/max_angular', 0.8)
		## maximum linear velocity with run button down (in m/s)
		# @param /max_linear_run (default: 0.55)
		self.max_lin_vel_run = rospy.get_param('/max_linear_run', 0.55)
		## maximum angular velocity with run button down (in rad/s)
		# @param /max_angular_run (default: 2.75)
		self.max_ang_vel_run = rospy.get_param('/max_angular_run', 2.75)
		## maximum scanning speed (in rad/s)
		# @param /max_scanning_speed (default: 1.20)
		self.max_scanning_speed = rospy.get_param('/max_scanning_speed', 1.20)
		## scanning speed increment when changing it (in rad/s)
		# @param ~scanning_speed_increment (default: 0.2)
		self.scanning_speed_increment = rospy.get_param('~scanning_speed_increment', 0.2)
		## flipper angle increment when moving them (in rad)
		# @param ~flipper_increment (default: 10*pi/180)
		self.flipper_increment = rospy.get_param('~flipper_increment', 10*pi/180.)
		# topic names
		# name of the velocity command topic of the robot driver
		command_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
		# name of the flipper command topic of the robot driver
		flippers_cmd_topic = rospy.get_param('~flippers_cmd_topic',
				'/flippers_cmd')
		# name of the flippers state topic published by the robot driver
		flippers_state_topic = rospy.get_param('~flippers_state_topic',
				'/flippers_state')
		# name of the brake command topic of the robot driver
		brake_topic = rospy.get_param('~brake_topic', '/brake')
		# name of the enable command topic of the robot driver
		enable_topic = rospy.get_param('~enable_topic', '/enable')
		# name of the scanning speed command topic of the robot driver
		scanning_speed_topic = rospy.get_param('~scanning_speed_topic', '/scanning_speed_cmd')
		# name of the robot status topic published by the robot driver
		robot_status_topic = rospy.get_param('~robot_status_topic', '/robot_status')
		# name of the joystick topic published by joy_node
		joy_topic = rospy.get_param('~joy_topic', '/joy')
		# publisher and subscribers
		## publisher for the velocity command topic
		# @param ~cmd_vel_topic (default: '/cmd_vel')
		self.cmdvel_pub = rospy.Publisher(command_topic, Twist)
		## publisher for the flipper command topic
		# @param ~flipper_cmd_topic (default: '/flippers_cmd')
		self.flippers_pub = rospy.Publisher(flippers_cmd_topic, FlippersState)
		## subscriber to the flippers state topic published by the robot driver
		# @param ~flippers_state_topic (default: '/flippers_state')
		rospy.Subscriber(flippers_state_topic, FlippersState, self.flippersCallBack)
		## publisher for the brake command topic
		# @param ~brake_topic (default: '/brake')
		self.brake_pub = rospy.Publisher(brake_topic, Bool)
		## subscriber to the robot status topic published by the robot driver
		# @param ~robot_status_topic (default: '/robot_status')
		rospy.Subscriber(robot_status_topic, RobotStatus, self.statusCallBack)
		## subscriber to the joystick topic published by joy_node
		# @param ~joy_topic (default: '/joy')
		rospy.Subscriber(joy_topic, Joy, self.joyCallBack)
		## publisher for the enable command topic
		# @param ~enable_topic (default: '/enable')
		self.enable_pub = rospy.Publisher(enable_topic, Bool)
		## publisher for the scanning speed command topic
		# @param ~scanning_speed_topic (default: '/scanning_speed_cmd')
		self.scanning_speed_pub = rospy.Publisher(scanning_speed_topic, Float64)


	## Listen to the status of the robot.
	def statusCallBack(self, robot_status):
		'''Listen to the status of the robot.'''
		## state of the brake (according to the driver)
		self.brake_on = robot_status.brake_on
		## scanning speed (according to the driver)
		self.scanning_speed = robot_status.scanning_speed

	## Listen to the flippers state.
	def flippersCallBack(self, flippers):
		'''Listen to the flippers state.'''
		## state of the flippers
		self.flippers = flippers
			

	## Callback for a joystick message.
	#
	# This method dispatches the message to each of the methods handling a
	# specific aspect:
	# - cmdvel_jcb,
	# - flipper_jcb,
	# - brake_jcb,
	# - enable_jcb,
	# - scanning_speed_jcb.
	def joyCallBack(self, joy):
		'''Callback for a joystick message.

		This method dispatches the message to each of the methods handling a
		specific aspect:
			- cmdvel_jcb,
			- flipper_jcb,
			- brake_jcb,
			- enable_jcb,
			- scanning_speed_jcb.'''
		self.joy.update(joy)
		self.cmdvel_jcb(self.joy)
		self.flipper_jcb(self.joy)
		self.brake_jcb(self.joy)
		self.enable_jcb(self.joy)
		self.scanning_speed_jcb(self.joy)


	## Handle velocity command based on the joystick input.
	def cmdvel_jcb(self, joy):
		'''Handle velocity command based on the joystick input.'''
		tw = Twist()
		if joy.buttons[self.deadman_button]:
			if joy.buttons[self.run_button]:
				lin_scale = self.max_lin_vel_run
				ang_scale = self.max_ang_vel_run
			else:
				lin_scale = self.max_lin_vel
				ang_scale = self.max_ang_vel
			tw.linear.x = lin_scale*joy.axes[self.lin_vel_axis]
			tw.angular.z = ang_scale*joy.axes[self.ang_vel_axis]
			self.cmdvel_pub.publish(tw)
		elif joy.released(self.deadman_button):	# make sure we ask the robot to stop
			tw.linear.x = 0.0
			tw.angular.z = 0.0
			self.cmdvel_pub.publish(tw)
		#else:
		#	pass	# we don't publish constantly


	## Handle flippers command based on the joystick input.
	def flipper_jcb(self, joy):
		'''Handle flippers command based on the joystick input.'''
		if joy.buttons[self.deadman_button]\
				and any([joy.buttons[self.flipper_button_fl],
						joy.buttons[self.flipper_button_fr],
						joy.buttons[self.flipper_button_rl],
						joy.buttons[self.flipper_button_rr]]) \
				and joy.axis_touched(self.flipper_axis):
			try:
				fs = FlippersState()
				fs.frontLeft = self.flippers.frontLeft
				fs.frontRight = self.flippers.frontRight
				fs.rearLeft = self.flippers.rearLeft
				fs.rearRight = self.flippers.rearRight
				if joy.buttons[self.flipper_button_fl]:
					fs.frontLeft += joy.axes[self.flipper_axis]*self.flipper_increment
				if joy.buttons[self.flipper_button_fr]:
					fs.frontRight += joy.axes[self.flipper_axis]*self.flipper_increment
				if joy.buttons[self.flipper_button_rl]:
					fs.rearLeft += joy.axes[self.flipper_axis]*self.flipper_increment
				if joy.buttons[self.flipper_button_rr]:
					fs.rearRight += joy.axes[self.flipper_axis]*self.flipper_increment
				#rospy.loginfo('d_FL: %f, d_FR: %f, d_RL:%f, d_RR:%f'%
				#		(fs.frontLeft - self.flippers.frontLeft,
				#		fs.frontRight - self.flippers.frontRight,
				#		fs.rearLeft - self.flippers.rearLeft,
				#		fs.rearRight - self.flippers.rearRight))
				self.flippers_pub.publish(fs)
			except AttributeError:
				rospy.logwarn('Flipper command ignored since no FlippersState message received.')


	## Handle brake command based on the joystick input.
	def brake_jcb(self, joy):
		'''Handle brake command based on the joystick input.'''
		if joy.buttons[self.deadman_button] and joy.pressed(self.brake_button):
			try:
				self.brake_pub.publish(not self.brake_on)
			except AttributeError:
				rospy.logwarn('Brake command ignored since no RobotStatus message received.')


	## Handle enable command based on the joystick input.
	def enable_jcb(self, joy):
		'''Handle enable command based on the joystick input.'''
		if joy.buttons[self.deadman_button] and joy.pressed(self.enable_button):
			self.enable_pub.publish(True)


	## Handle scanning speed command based on the joystick input.
	def scanning_speed_jcb(self, joy):
		'''Handle scanning speed command based on the joystick input.'''
		if joy.buttons[self.deadman_button] and\
				joy.axis_touched(self.scanning_speed_axis):
			try:
				v = min(self.max_scanning_speed,
						abs(self.scanning_speed+self.scanning_speed_increment*joy.axes[self.scanning_speed_axis]))
				self.scanning_speed_pub.publish(v)
			except AttributeError:
				rospy.logwarn('Scanning speed change command ignored since no\
				RobotStatus message receive.')
################################################################################



## Class to allow detecting transition in buttons and axes.
#
# It can be used like a Joy message but with additional methods.
class HistoryJoystick(Joy):
	'''Class to allow detecting transition in buttons and axes.'''

	## Instantiate a HistoryJoystick object
	def __init__(self):
		## Current state of the buttons
		self.buttons = None
		## Current state of the axes
		self.axes = None

	## To be called with each new joystick data.
	def update(self, joy):
		'''To be called with each new joystick data.'''
		## Previous state of the buttons
		self.old_buttons = self.buttons
		## Previous state of the axes
		self.old_axes = self.axes
		self.buttons = joy.buttons
		self.axes = joy.axes

	## Check if a given button is currently pressed down (state 1)
	def is_down(self, button_id):
		'''Check if a given button is currently pressed down (state 1).'''
		try:
			return self.buttons[button_id]
		except TypeError:
			# no joystick messages yet?
			return False

	## Check if a given button has just been pressed (transition 0->1).
	def pressed(self, button_id):
		'''Check if a given button has just been pressed (transition 0->1).'''
		try:
			return self.buttons[button_id] and\
					not self.old_buttons[button_id]
		except (AttributeError, TypeError), e:
			# not enough joystick messages yet?
			return False
	
	## Check if a given button has just been released (transition 1->0).
	def released(self, button_id):
		'''Check if a given button has just been released (transition 1->0).'''
		try:
			return not self.buttons[button_id] and\
					self.old_buttons[button_id]
		except (AttributeError, TypeError), e:
			# not enough joystick messages yet?
			return False
	
	## Check if a given button state has just changed (either transitions).
	def button_changed(self, button_id):
		'''Check if a given button state has just changed (either transitions).'''
		try:
			return self.buttons[button_id] != \
					self.old_buttons[button_id]
		except (TypeError, AttributeError), e:
			# not enough joystick messages yet?
			return False

	## Check if a given axis has moved.
	def axis_moved(self, axis_id):
		'''Check if a given axis has moved.'''
		try:
			return self.axes[axis_id] != \
					self.old_axes[axis_id]
		except (TypeError, AttributeError), e:
			# no joystick messages yet?
			return False

	## Check if a given axis that was released has moved (transition 0->anything
	# else).
	def axis_touched(self, axis_id):
		'''Check if a given axis that was released has moved (transition
		0->anything else).'''
		try:
			return self.axes[axis_id] and \
					not self.old_axes[axis_id]
		except (TypeError, AttributeError), e:
			# no joystick messages yet?
			return False

	## Check if a given axis has just been released (transition anything
	# else->0).
	def axis_touched(self, axis_id):
		'''Check if a given axis has just been released has (transition anything
		else->0).'''
		try:
			return not self.axes[axis_id] and \
					self.old_axes[axis_id]
		except (TypeError, AttributeError), e:
			# no joystick messages yet?
			return False
################################################################################



## Create a ROS node and instantiate the NiftiTeleopJoy class.
def main():
	'''Create a ROS node and instantiate the NiftiTeleopJoy class.'''
	try:
		rospy.init_node('nifti_teleop_joy')
		ntj = NiftiTeleopJoy()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
		
