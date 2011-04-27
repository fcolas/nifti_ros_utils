#!/usr/bin/env python
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from geometry_msgs.msg import Twist
from joy.msg import Joy
from nifti_ros_drivers.msg import FlippersState, Brake, RobotStatus
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
		self.brake_button = rospy.get_param('~differential_brake_button', 8)
		self.flipper_button_fl = rospy.get_param('~front_left_flipper_button', 4)
		self.flipper_button_fr = rospy.get_param('~front_right_flipper_button', 5)
		self.flipper_button_rl = rospy.get_param('~rear_left_flipper_button', 6)
		self.flipper_button_rr = rospy.get_param('~rear_right_flipper_button', 7)
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
		self.flippers_ok = False
		brake_topic = rospy.get_param('~teleop_brake', '/brake')
		rospy.Subscriber('flippers_state', FlippersState, self.flippersCallBack)
		self.brake_pub = rospy.Publisher(brake_topic, Brake)
		self.brake_ok = False
		rospy.Subscriber('robot_status', RobotStatus, self.statusCallBack)
		rospy.Subscriber('joy', Joy, self.joyCallBack)


	def statusCallBack(self, robot_status):
		self.brake_on = robot_status.brake.on
		self.brake_ok = True


	def flippersCallBack(self, flippers):
		self.flippers = flippers
		self.flippers_ok = True
			

	def joyCallBack(self, joy):
		self.cmdvel_cb(joy)
		self.flipper_cb(joy)
		self.brake_cb(joy)
	

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
				and any([joy.buttons[self.flipper_button_fl],
						joy.buttons[self.flipper_button_fr],
						joy.buttons[self.flipper_button_rl],
						joy.buttons[self.flipper_button_rr]]) \
				and joy.axes[self.flipper_axis]:
			if not self.flippers_ok:
				rospy.logwarn('Flipper command disabled since no FlippersState message received.')
			else:
				fs = FlippersState(self.flippers.frontLeft, self.flippers.frontRight,
						self.flippers.rearLeft, self.flippers.rearRight)
				if joy.buttons[self.flipper_button_fl]:
					fs.frontLeft += joy.axes[self.flipper_axis]*self.flipper_increment
				if joy.buttons[self.flipper_button_fr]:
					fs.frontRight += joy.axes[self.flipper_axis]*self.flipper_increment
				if joy.buttons[self.flipper_button_rl]:
					fs.rearLeft += joy.axes[self.flipper_axis]*self.flipper_increment
				if joy.buttons[self.flipper_button_rr]:
					fs.rearRight += joy.axes[self.flipper_axis]*self.flipper_increment
				self.flippers_pub.publish(fs)
		#else:
		#	pass	# we don't publish constantly

	
	def brake_cb(self, joy):
		if joy.buttons[self.deadman_button] and joy.buttons[self.brake_button]:
			if not self.brake_ok:
				rospy.logwarn('Flipper command disabled since no RobotStatus message received.')
			else:
				self.brake_pub.publish(not self.brake_on)
		#else:
		#	pass



def main():
	try:
		rospy.init_node('nifti_teleop_joy')
		ntj = NiftiTeleopJoy()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
		
