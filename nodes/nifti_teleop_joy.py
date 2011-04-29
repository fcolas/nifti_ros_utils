#!/usr/bin/env python
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from joy.msg import Joy
from nifti_ros_drivers.msg import FlippersState, RobotStatus
from math import pi


class NiftiTeleopJoy(object):

	def __init__(self):
		# joystick
		self.joy = HistoryJoystick()
		# joystick axes
		self.lin_vel_axis = rospy.get_param('~axis_linear', 1)
		self.ang_vel_axis = rospy.get_param('~axis_angular', 0)
		self.flipper_axis = rospy.get_param('~flipper_axis', 5)
		self.scanning_speed_axis = rospy.get_param('~scanning_speed_axis', 4)
		# joystick buttons
		self.deadman_button = rospy.get_param('~deadman_button', 1)
		self.run_button = rospy.get_param('~run_button', 0)
		self.enable_button = rospy.get_param('~enable_button', 9)
		self.brake_button = rospy.get_param('~differential_brake_button', 8)
		self.flipper_button_fl = rospy.get_param('~front_left_flipper_button', 4)
		self.flipper_button_fr = rospy.get_param('~front_right_flipper_button', 5)
		self.flipper_button_rl = rospy.get_param('~rear_left_flipper_button', 6)
		self.flipper_button_rr = rospy.get_param('~rear_right_flipper_button', 7)
		# speed limits
		self.max_lin_vel = rospy.get_param('/max_linear', 0.3)
		self.max_ang_vel = rospy.get_param('/max_angular', 0.8)
		self.max_lin_vel_run = rospy.get_param('/max_linear_run', 0.55)
		self.max_ang_vel_run = rospy.get_param('/max_angular_run', 2.75)
		self.max_scanning_speed = rospy.get_param('/max_scanning_speed', 1.20)
		self.scanning_speed_increment = rospy.get_param('~scanning_speed_increment', 0.2)
		self.flipper_increment = rospy.get_param('~flipper_increment', 10*pi/180.)
		# topic names
		command_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
		flippers_cmd_topic = rospy.get_param('~flippers_cmd_topic',
				'/flippers_cmd')
		flippers_state_topic = rospy.get_param('~flippers_state_topic',
				'/flippers_state')
		brake_topic = rospy.get_param('~brake_topic', '/brake')
		enable_topic = rospy.get_param('~enable_topic', '/enable')
		scanning_speed_topic = rospy.get_param('~scanning_speed_topic', '/scanning_speed_cmd')
		robot_status_topic = rospy.get_param('~robot_status_topic', '/robot_status')
		joy_topic = rospy.get_param('~joy_topic', '/joy')
		# publisher and subscribers
		self.cmdvel_pub = rospy.Publisher(command_topic, Twist)
		self.flippers_pub = rospy.Publisher(flippers_cmd_topic, FlippersState)
		rospy.Subscriber(flippers_state_topic, FlippersState, self.flippersCallBack)
		self.brake_pub = rospy.Publisher(brake_topic, Bool)
		rospy.Subscriber(robot_status_topic, RobotStatus, self.statusCallBack)
		rospy.Subscriber(joy_topic, Joy, self.joyCallBack)
		self.enable_pub = rospy.Publisher(enable_topic, Bool)
		self.scanning_speed_pub = rospy.Publisher(scanning_speed_topic, Float64)


	def statusCallBack(self, robot_status):
		self.brake_on = robot_status.brake_on
		self.scanning_speed = robot_status.scanning_speed


	def flippersCallBack(self, flippers):
		self.flippers = flippers
			

	def joyCallBack(self, joy):
		self.joy.update(joy)
		self.cmdvel_jcb(self.joy)
		self.flipper_jcb(self.joy)
		self.brake_jcb(self.joy)
		self.enable_jcb(self.joy)
		self.scanning_speed_jcb(self.joy)
	

	def cmdvel_jcb(self, joy):
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

			
	def flipper_jcb(self, joy):
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

	
	def brake_jcb(self, joy):
		if joy.buttons[self.deadman_button] and joy.pressed(self.brake_button):
			try:
				self.brake_pub.publish(not self.brake_on)
			except AttributeError:
				rospy.logwarn('Brake command ignored since no RobotStatus message received.')


	def enable_jcb(self, joy):
		if joy.buttons[self.deadman_button] and joy.pressed(self.enable_button):
			self.enable_pub.publish(True)


	def scanning_speed_jcb(self, joy):
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



# joystick helper
class HistoryJoystick(Joy):
	'''Class to allow detecting transition in buttons and axes.'''

	def __init__(self):
		self.buttons = None
		self.axes = None

	def update(self, joy):
		'''To be called with each new joystick data.'''
		self.old_buttons = self.buttons
		self.old_axes = self.axes
		self.buttons = joy.buttons
		self.axes = joy.axes

	def is_down(self, button_id):
		'''Check if a given button is currently pressed down.'''
		try:
			return self.buttons[button_id]
		except AttributeError:
			# no joystick messages yet?
			return False

	def pressed(self, button_id):
		'''Check if a given button has just been pressed.'''
		try:
			return self.buttons[button_id] and\
					not self.old_buttons[button_id]
		except AttributeError:
			# not enough joystick messages yet?
			return False
		
	def released(self, button_id):
		'''Check if a given button has just been released.'''
		try:
			return not self.buttons[button_id] and\
					self.old_buttons[button_id]
		except AttributeError:
			# not enough joystick messages yet?
			return False
	
	def button_changed(self, button_id):
		'''Check if a given button state has just changed.'''
		try:
			return self.buttons[button_id] != \
					self.old_buttons[button_id]
		except AttributeError:
			# not enough joystick messages yet?
			return False

	def axis_moved(self, axis_id):
		'''Check if a given axis has moved.'''
		try:
			return self.axes[axis_id] != \
					self.old_axes[axis_id]
		except AttributeError:
			# no joystick messages yet?
			return False

	def axis_touched(self, axis_id):
		'''Check if a given axis that was released has moved.'''
		try:
			return self.axes[axis_id] and \
					not self.old_axes[axis_id]
		except AttributeError:
			# no joystick messages yet?
			return False




def main():
	'''Creates a node and instantiate the NiftiTeleopJoy class.'''
	try:
		rospy.init_node('nifti_teleop_joy')
		ntj = NiftiTeleopJoy()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
		
