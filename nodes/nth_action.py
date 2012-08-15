#!/usr/bin/env python
## @file
# Set of helper tools for the teleoperation of the NIFTi robot
#
# Right now only ScanningService is implemented: it listens on a topic and
# handles the motion of the rolling laser.
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from rospy.rostime import get_time
from std_msgs.msg import Bool, Float64, Int32
from sensor_msgs.msg import PointCloud2
from nifti_robot_driver_msgs.msg import RobotStatusStamped, FlippersState,\
		FlippersStateStamped
import tf

import actionlib
from actionlib.server_goal_handle import ServerGoalHandle
from actionlib_msgs.msg import GoalStatus
from nifti_teleop.msg import *

from math import atan2, pi, radians

class FlipperPosture(object):
	'''Handle the configuration of the flippers.
	'''
	postures={
		# 0: flat
		0: (0, 0, 0, 0),
		# 1: drive
		1: (radians(-165), radians(-165), radians(115), radians(115)),
		# 2: climb forward
		2: (radians(-45), radians(-45), radians(0), radians(0)),
		# 3: climb backward
		3: (radians(0), radians(0), radians(45), radians(45)),
		# 4: convex edge
		4: (radians(40), radians(40), radians(-40), radians(-40)),
		# 5: bestInit
		5: (radians(-120), radians(-120), radians(120), radians(120))}
		

	def __init__(self):
		self.flippers_pub = rospy.Publisher('/flippers_cmd', FlippersState)
		self.posture_pub = rospy.Publisher('/posture', Int32)
		rospy.Subscriber('/flippers_state', FlippersStateStamped,
				self.flippers_cb)
		rospy.Subscriber('/posture_cmd', Int32, self.cmd_cb)

	def cmd_cb(self, posture):
		pid = posture.data
#		rospy.logdebug("NTH - Received posture command: %d"%pid)
		try:
			fl, fr, rl, rr = self.postures[pid]
			fs = FlippersState()
			fs.frontLeft = fl
			fs.frontRight = fr
			fs.rearLeft = rl
			fs.rearRight = rr
			self.flippers_pub.publish(fs)
		except IndexError:
			rospy.logerr("NTH - Unknow posture id: %d"%pid)
	
	def flippers_cb(self, flippers):
		fl = flippers.frontLeft
		fr = flippers.frontRight
		rl = flippers.rearLeft
		rr = flippers.rearRight
		tol = radians(5)
		ret_pid = -1
		for pid, (pfl, pfr, prl, prr) in self.postures.items():
			if abs(fl-pfl)<=tol and abs(fr-pfr)<=tol and abs(rl-prl)<=tol\
					and abs(rr-prr)<=tol:
				ret_pid = pid
				break
		self.posture_pub.publish(ret_pid)


## ScanningService class
class ScanningService(object):
	'''Handle the motion of the rolling laser for taking a single 3D scan.
	'''

	## Instantiate the ScanningService class
	def __init__(self):
		## scanning speed command publisher
		self.scanning_speed_pub = rospy.Publisher('/scanning_speed_cmd', Float64)
		## laser center command publisher
		self.laser_center_pub = rospy.Publisher('/laser_center', Bool)
		## robot status subscriber (to get the current speed of the laser)
		rospy.Subscriber('/robot_status', RobotStatusStamped, self.status_cb)
		## scanning_once subscriber
		rospy.Subscriber('/scanning_once', Float64, self.scanning_once_cb)
		## point cloud control publisher
		self.ptcld_ctrl_pub = rospy.Publisher('/pointcloud_control', Bool)
		## scanning state
		self.scanning_state = ScanningFeedback.READY
		## last goal time set, to avoid race condition
		self.last_goal_time = get_time()
		## action server
		self.goal = ServerGoalHandle()
		self.action_server = actionlib.ActionServer('ScanningOnceAS',
				ScanningAction, self.goal_cb, auto_start=False)
		self.action_server.start()
		## action client
		self.action_client = actionlib.SimpleActionClient('ScanningOnceAS',
				ScanningAction)

		## tf listener for laser scanner angle
		self.tf_listener = tf.TransformListener()

	
	## robot status callback to get the current speed of the laser
	def status_cb(self, robot_status):
		self.scanning_speed = robot_status.scanning_speed
		if (self.scanning_speed == 0.0) and \
				(get_time() - self.last_goal_time>0.5):	
			# if not moving, updating state
			rospy.logdebug('NTH - Laser speed 0: setting state to "Not scanning".')
			if self.scanning_state != ScanningFeedback.READY:
				self.scanning_state = ScanningFeedback.READY
				self.goal.set_succeeded(ScanningResult(ScanningResult.ERROR),
						"Aborting as laser is not moving")
				self.goal.publish_feedback(ScanningFeedback(self.scanning_state))


	def cancel_cb(self, goal):
		if goal==self.goal:
			self.stop_scanning()	# Temporary

	## callback when a goal is received
	def goal_cb(self, goal):
		if self.goal.get_goal() and \
				self.goal.get_goal_status().status == GoalStatus.ACTIVE:
			goal.set_rejected(ScanningResult(ScanningResult.WARNING),
				"Can only do one scan at a time")
			return
		scanning_goal = goal.get_goal()
		if scanning_goal.action == ScanningGoal.START_SCANNING:
			if self.scanning_state == ScanningFeedback.READY and\
					self.scanning_speed==0:
				self.goal = goal
				self.goal.set_accepted("Scan accepted")
				self.start_scanning(scanning_goal.speed)
			else:
				goal.set_rejected(ScanningResult(ScanningResult.ERROR),
						"Already scanning")
		elif scanning_goal.action == ScanningGoal.STOP_SCANNING:
			self.goal = goal
			self.goal.set_accepted("Stop accepted")
			self.stop_scanning()
		else:
			goal.set_rejected(ScanningResult(ScanningResult.ERROR),
					"Unknown action")

	def stop_scanning(self):
		rospy.loginfo("NTH - Stopping and centering laser")
		self.laser_center_pub.publish(True)
		if self.goal.get_goal().action==ScanningGoal.STOP_SCANNING:
			goal.set_succeeded(ScanningResult(ScanningResult.SUCCESS),
					"Success in stopping laser")
		else:
			goal.set_succeeded(ScanningResult(ScanningResult.WARNING),
					"Scan aborted")
		self.scanning_state = ScanningFeedback.READY
		goal.publish_feedback(ScanningFeedback(self.scanning_state))

	## forwarding scanning_once topic to a new goal
	def scanning_once_cb(self, speed):

		goal = ScanningGoal()
		goal.action = ScanningGoal.START_SCANNING
		goal.speed = speed.data
		self.action_client.send_goal(goal)

	def start_scanning(self, speed):
		# clip speed
		speed = max(min(speed, 0.1), 1.2)
		# send command
		rospy.loginfo("NTH - Starting scan")
		rospy.logdebug("NTH - Sending scanning speed command: %f and disabling \
publication of the messy point cloud."%speed)
		self.ptcld_ctrl_pub.publish(False)
		self.scanning_speed_pub.publish(speed)
		self.last_goal_time = get_time()
		# update state
		self.scanning_state = ScanningFeedback.WAITING_FOR_FIRST_SWIPE
		self.goal.publish_feedback(ScanningFeedback(self.scanning_state))
	
	def run(self):
		r = rospy.Rate(15)
		last_angle = 0
		last_direction = 0
		direction = 0
		last_tf = rospy.Time(0)
		while not rospy.is_shutdown():
			r.sleep()
			try:
				tf_time = self.tf_listener.getLatestCommonTime("/base_link",
						"/laser")
				if tf_time==last_tf:
					continue
				_, (x, _, _, w) = self.tf_listener.lookupTransform("/base_link", "/laser",
					tf_time)
				angle = atan2(-2*x*w, 2*x*x-1)
				if angle>last_angle:
					direction = 1
				elif angle<last_angle:
					direction = -1
				#print last_angle, angle, last_direction, direction, self.scanning_speed
				if self.scanning_speed*direction*last_direction < 0 and\
						abs(angle)>pi/3:	# TODO might be problematic
					# End of swipe event
					rospy.logdebug("NTH - End of swipe")
					if self.scanning_state == ScanningFeedback.WAITING_FOR_FIRST_SWIPE:
						# no end of swipe received before
						rospy.logdebug("NTH - Detected first end of swipe: waiting for a \
second one and activating point cloud publication.")
						self.ptcld_ctrl_pub.publish(True)
						self.scanning_state = ScanningFeedback.WAITING_FOR_COMPLETE_SWIPE
						self.goal.publish_feedback(ScanningFeedback(self.scanning_state))
					elif self.scanning_state == ScanningFeedback.WAITING_FOR_COMPLETE_SWIPE:
						# first end of swipe received before
						rospy.logdebug("NTH - Detected second end of swipe: stopping laser and \
centering it.")
						self.scanning_state = ScanningFeedback.READY
						#self.scanning_speed_pub.publish(0.0) # may be unnecessary
						self.laser_center_pub.publish(True)
						self.goal.publish_feedback(ScanningFeedback(self.scanning_state))
						self.goal.set_succeeded(ScanningResult(ScanningResult.SUCCESS),
								"Scan succeeded")
						rospy.loginfo("NTH - Scan ended")
					else:
						rospy.logdebug("NTH - end of swipe received and ignored.")
				last_angle = angle
				last_direction = direction
				last_tf = tf_time
			except tf.Exception, e:
				rospy.logwarn("NTH - Exception: %s"%str(e))




def main():
	try:
		# starts the node
		rospy.init_node('nth')
		# instantiate the class
		scan3d = ScanningService()
		fp = FlipperPosture()
		# wait until closed
		scan3d.run()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	

