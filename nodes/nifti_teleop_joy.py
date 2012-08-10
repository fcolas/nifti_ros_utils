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
# diamonback:
#from joy.msg import Joy
# electric and later:
from sensor_msgs.msg import Joy

from nifti_robot_driver_msgs.msg import FlippersState, RobotStatusStamped, FlipperCommand, FlippersStateStamped
from nifti_teleop.srv import Acquire, Release

from math import pi, floor, radians

# TODO: should be part of the message definition
ID_FLIPPER_FRONT_LEFT=3
ID_FLIPPER_FRONT_RIGHT=4
ID_FLIPPER_REAR_LEFT=5
ID_FLIPPER_REAR_RIGHT=6

## Joystick teleoperation class
class NiftiTeleopJoy(object):
	'''Joystick teleoperation interface.

	It handles velocity command of the platform, activation of the brake, motion
	of the flippers, and selection of the rolling speed of the laser scanner.
	'''
	
	## Instantiate a NiftiTeleopJoy object
	def __init__(self):
		## current flippers state
		self.fs = FlippersState()
		self.fs.frontLeft = None 
		self.fs.frontRight = None
		self.fs.rearLeft = None
		self.fs.rearRight = None

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
		## button to reset flippers to flat position
		# @param ~flipper_reset_button (default: 3)
		self.flipper_reset_button = rospy.get_param('~flipper_reset_button', 3)
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
		## scanning once button
		# @param ~scanning_once_button (default: 2)
		self.scanning_once_button = rospy.get_param('~scanning_once_button', 2)
		## mapping toggle button
		# @param ~mapping_toggle_button (default: 11)
		self.mapping_toggle_button = rospy.get_param('~mapping_toggle_button', 11)

		# speed limits
		## maximum linear velocity (in m/s)
		# @param /max_linear (default: 0.3)
		self.max_lin_vel = rospy.get_param('/max_linear', 0.3)
		## maximum angular velocity (in rad/s)
		# @param /max_angular (default: 0.6)
		self.max_ang_vel = rospy.get_param('/max_angular', 0.6)
		## maximum linear velocity with run button down (in m/s)
		# @param /max_linear_run (default: 0.6)
		self.max_lin_vel_run = rospy.get_param('/max_linear_run', 0.6)
		## maximum angular velocity with run button down (in rad/s)
		# @param /max_angular_run (default: 1.24)
		self.max_ang_vel_run = rospy.get_param('/max_angular_run', 1.24)
		## maximum scanning speed (in rad/s)
		# @param /max_scanning_speed (default: 1.20)
		self.max_scanning_speed = rospy.get_param('/max_scanning_speed', 1.20)
		## tracks distance (in m)
		# @param /tracks_distance (default: 0.397)
		self.tracks_distance = rospy.get_param('/tracks_distance', 0.397)
		## steering efficiency
		# @param /steering_efficiency (default: 0.41)
		self.steering_efficiency = rospy.get_param('/steering_efficiency', 0.41)
		## scanning speed increment when changing it (in rad/s)
		# @param ~scanning_speed_increment (default: 0.2)
		self.scanning_speed_increment = rospy.get_param('~scanning_speed_increment', 0.2)
		## flipper angle increment when moving them (in rad)
		# @param ~flipper_increment (default: 10*pi/180)
		self.flipper_increment = rospy.get_param('~flipper_increment', 20*pi/180.)
		## scanning once speed (in rad/s)
		# @param /scanning_once_speed (default: 0.60)
		self.scanning_once_speed = rospy.get_param('/scanning_once_speed', 1.20)

		## cmd_vel topic
		self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic',
				'/teleop_joy/cmd_vel')

		## current mapping state
		self.mapping_on = True

		## last front command
		self.last_front_time = rospy.Time(0)
		## last rear command
		self.last_rear_time = rospy.Time(0)

		# publishers
		## publisher for the velocity command topic
		# @param ~cmd_vel_topic (default: '/teleop_joy/cmd_vel')
		self.cmdvel_pub = rospy.Publisher(self.cmd_vel_topic, Twist)
		## publisher for the all flippers command topic
		self.flippers_pub = rospy.Publisher('/flippers_cmd', FlippersState)
		## publisher for the individual flipper command topic
		self.flipper_pub = rospy.Publisher('/flipper_cmd', FlipperCommand)
		## publisher for the brake command topic
		self.brake_pub = rospy.Publisher('/brake', Bool)
		## publisher for the enable command topic
		self.enable_pub = rospy.Publisher('/enable', Bool)
		## publisher for the scanning speed command topic
		self.scanning_speed_pub = rospy.Publisher('/scanning_speed_cmd', Float64)
		## publisher for the laser centering command topic
		self.laser_center_pub = rospy.Publisher('/laser_center', Bool)
		## publisher for the scanning once topic
		self.scanning_once_pub = rospy.Publisher('/scanning_once', Float64)
		## publisher for the mapping control topic
		self.mapping_control_pub = rospy.Publisher('/mapping_control', Bool)

		# setting up priority requests
		gotit = False
		while not gotit:
			try:
				rospy.wait_for_service('/mux_cmd_vel/acquire', 10)
				rospy.wait_for_service('/mux_cmd_vel/release', 10)
				gotit = True
			except rospy.ROSException:
				rospy.logwarn("Waiting for mux control services...")
		self.priority_acquire = rospy.ServiceProxy('/mux_cmd_vel/acquire',
				Acquire)
		self.priority_release = rospy.ServiceProxy('/mux_cmd_vel/release',
				Release)

		# subscribers
		## subscriber to the flippers state topic published by the robot driver
		# @param ~flippers_state_topic (default: '/flippers_state')
		rospy.Subscriber('/flippers_state', FlippersStateStamped, self.flippersCallBack)
		## subscriber to the steering_efficiency topic
		rospy.Subscriber('/steering_efficiency', Float64, self.steering_efficiency_cb)
		## subscriber to the robot status topic published by the robot driver
		rospy.Subscriber('/robot_status', RobotStatusStamped, self.statusCallBack)
		## subscriber to the joystick topic published by joy_node
		rospy.Subscriber('/joy', Joy, self.joyCallBack)



	## Update the steering efficiency
	def steering_efficiency_cb(self, msg):
		'''Update the steering efficiency.'''
		self.steering_efficiency = max(0.0, min(1.0, msg.data))

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
		# state of the flippers
		#self.flippers = flippers

		def roundFlipperValue(angle, inc=1.0*self.flipper_increment):
			return inc*round(angle/inc)
		self.fs.frontLeft = roundFlipperValue(flippers.frontLeft)
		self.fs.frontRight = roundFlipperValue(flippers.frontRight)
		self.fs.rearLeft = roundFlipperValue(flippers.rearLeft)
		self.fs.rearRight = roundFlipperValue(flippers.rearRight)


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
		self.scanning_once_jcb(self.joy)
		self.mapping_control_jcb(self.joy)
		self.easy_flipper_jcb(self.joy)

		self.mux_jcb(self.joy)

	#TODO test and parametrize correctly
	def easy_flipper_jcb(self, joy):
		fb = -joy.axes[2]
		ud = joy.axes[3]
		if joy.pressed(11):
			fs = FlippersState()
			if fb>0.5 and ud>0.5:
				fs.frontLeft = radians(-40)
				fs.frontRight = radians(-40)
				fs.rearLeft = radians(0)
				fs.rearRight = radians(0)
				self.flippers_pub.publish(fs)
			if fb>0.5 and ud<-0.5:
				fs.frontLeft = radians(50)
				fs.frontRight = radians(50)
				fs.rearLeft = radians(-30)
				fs.rearRight = radians(-30)
				self.flippers_pub.publish(fs)
			if fb<-0.5 and ud>0.5:
				fs.frontLeft = radians(0)
				fs.frontRight = radians(0)
				fs.rearLeft = radians(40)
				fs.rearRight = radians(40)
				self.flippers_pub.publish(fs)
			if fb<-0.5 and ud<-0.5:
				fs.frontLeft = radians(30)
				fs.frontRight = radians(30)
				fs.rearLeft = radians(-50)
				fs.rearRight = radians(-50)
				self.flippers_pub.publish(fs)
			if abs(fb)<0.2 and abs(ud)<0.2:
				fs.frontLeft = radians(0)
				fs.frontRight = radians(0)
				fs.rearLeft = radians(0)
				fs.rearRight = radians(0)
				self.flippers_pub.publish(fs)
		elif joy.buttons[self.deadman_button]:
			if ud<-0.5:
				flipper_change = -self.flipper_increment
			elif ud>0.5:
				flipper_change = self.flipper_increment
			else:
				return
			flipperMotion = FlipperCommand()
			now = rospy.Time.now()
			if fb>0.5:
				if (now-self.last_front_time).to_sec()>0.5:
					rospy.loginfo("Moving front")
					self.last_front_time = now
					flipperMotion.object_id = ID_FLIPPER_FRONT_LEFT
					flipperMotion.angle = self.fs.frontLeft + flipper_change
					self.flipper_pub.publish(flipperMotion)
					flipperMotion.object_id = ID_FLIPPER_FRONT_RIGHT
					flipperMotion.angle = self.fs.frontRight + flipper_change
					self.flipper_pub.publish(flipperMotion)
			elif fb<-0.5:
				if (now-self.last_rear_time).to_sec()>0.5:
					self.last_rear_time = now
					flipperMotion.object_id = ID_FLIPPER_REAR_LEFT
					flipperMotion.angle = self.fs.rearLeft + flipper_change
					self.flipper_pub.publish(flipperMotion)
					flipperMotion.object_id = ID_FLIPPER_REAR_RIGHT
					flipperMotion.angle = self.fs.rearRight + flipper_change
					self.flipper_pub.publish(flipperMotion)




	## Handle priority based on the joystick input.
	def mux_jcb(self, joy):
		if joy.pressed(self.deadman_button):
			self.priority_acquire(self.cmd_vel_topic)
		elif joy.released(self.deadman_button):
			self.priority_release(self.cmd_vel_topic)



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
			# limit tracks speed
			v = lin_scale*joy.axes[self.lin_vel_axis]
			w = ang_scale*joy.axes[self.ang_vel_axis]
			l = v - w*self.tracks_distance/2./self.steering_efficiency
			r = v + w*self.tracks_distance/2./self.steering_efficiency
			Z = 1.
			if abs(l)>self.max_lin_vel_run:
				Z = abs(l)/self.max_lin_vel_run
			if abs(r)>self.max_lin_vel_run:
				Z = abs(r)/self.max_lin_vel_run
			r = r / Z
			l = l / Z
			v = (l + r)/2.
			w = (r - l)*self.steering_efficiency/self.tracks_distance

			tw.linear.x = v
			tw.angular.z = w

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
		if joy.buttons[self.deadman_button]:
			if joy.pressed(self.flipper_reset_button):	# flat button
				self.fs.frontLeft = 2*pi*floor((self.fs.frontLeft+1.5*pi)/(2*pi))
				self.fs.frontRight = 2*pi*floor((self.fs.frontRight+1.5*pi)/(2*pi))
				self.fs.rearLeft = 2*pi*floor((self.fs.rearLeft+0.5*pi)/(2*pi))
				self.fs.rearRight = 2*pi*floor((self.fs.rearRight+0.5*pi)/(2*pi))
				self.flippers_pub.publish(self.fs)
			elif joy.axis_touched(self.flipper_axis):
				try:
					if all([joy.buttons[self.flipper_button_fl],
							joy.buttons[self.flipper_button_fr],
							joy.buttons[self.flipper_button_rl],
							joy.buttons[self.flipper_button_rr]]):
						self.fs.frontLeft -= joy.axes[self.flipper_axis]*self.flipper_increment
						self.fs.frontRight -= joy.axes[self.flipper_axis]*self.flipper_increment
						self.fs.rearLeft += joy.axes[self.flipper_axis]*self.flipper_increment
						self.fs.rearRight += joy.axes[self.flipper_axis]*self.flipper_increment
						self.flippers_pub.publish(self.fs)
					else:	# individual control
						flipperMotion = FlipperCommand()
						if joy.buttons[self.flipper_button_fl]:
							flipperMotion.object_id = ID_FLIPPER_FRONT_LEFT
							flipperMotion.angle = self.fs.frontLeft - joy.axes[self.flipper_axis]*self.flipper_increment
							self.flipper_pub.publish(flipperMotion)
						if joy.buttons[self.flipper_button_fr]:
							flipperMotion.object_id = ID_FLIPPER_FRONT_RIGHT
							flipperMotion.angle = self.fs.frontRight - joy.axes[self.flipper_axis]*self.flipper_increment
							self.flipper_pub.publish(flipperMotion)
						if joy.buttons[self.flipper_button_rl]:
							flipperMotion.object_id = ID_FLIPPER_REAR_LEFT
							flipperMotion.angle = self.fs.rearLeft + joy.axes[self.flipper_axis]*self.flipper_increment
							self.flipper_pub.publish(flipperMotion)
						if joy.buttons[self.flipper_button_rr]:
							flipperMotion.object_id = ID_FLIPPER_REAR_RIGHT
							flipperMotion.angle = self.fs.rearRight + joy.axes[self.flipper_axis]*self.flipper_increment
							self.flipper_pub.publish(flipperMotion)
				except (TypeError, AttributeError), e:
					rospy.logwarn('Flipper command ignored since no FlippersState message received.')


	## Handle brake command based on the joystick input.
	def brake_jcb(self, joy):
		'''Handle brake command based on the joystick input.'''
		if joy.buttons[self.deadman_button] and joy.pressed(self.brake_button):
			try:
				self.brake_pub.publish(not self.brake_on)
			except AttributeError:
				rospy.logwarn('Brake command ignored since no RobotStatus message received.')


	## Handle scanning_once command based on the joystick input.
	def scanning_once_jcb(self, joy):
		'''Handle scanning_once command based on the joystick input.'''
		if joy.buttons[self.deadman_button] and joy.pressed(self.scanning_once_button):
			rospy.loginfo('Initiating 3D scan at speed %f rad/s.'\
					%self.scanning_once_speed)
			self.scanning_once_pub.publish(self.scanning_once_speed)


	## Handle scanning_once command based on the joystick input.
	def mapping_control_jcb(self, joy):
		'''Handle mapping_control command based on the joystick input.'''
		if joy.buttons[self.deadman_button] and joy.pressed(self.mapping_toggle_button):
			if self.mapping_on:
				rospy.loginfo('Disabling mapping.')
			else:
				rospy.loginfo('Enabling mapping.')
			self.mapping_control_pub.publish(not self.mapping_on)
			self.mapping_on = not self.mapping_on


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
				v = self.scanning_speed-self.scanning_speed_increment*joy.axes[self.scanning_speed_axis]
				if v<0.:
					v = 0.
					self.laser_center_pub.publish(True)
				else:
					v = min(self.max_scanning_speed, v)
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
		## Header
		self.header = None

	## To be called with each new joystick data.
	def update(self, joy):
		'''To be called with each new joystick data.'''
		## Previous state of the buttons
		self.old_buttons = self.buttons
		## Previous state of the axes
		self.old_axes = self.axes
		self.buttons = joy.buttons
		self.axes = joy.axes
		self.header = joy.header

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
	def axis_released(self, axis_id):
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
	
	
