#!/usr/bin/env python
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from topic_tools.srv import MuxSelect
from nifti_teleop.srv import *
from std_msgs.msg import String

from threading import Lock
from time import sleep, clock

## priority definition
# higher values have higher priority
priority = {
	"__none": -1,
	"/nav/cmd_vel": 0,
	"/ocu/cmd_vel": 2,
	"/nifti_teleop_joy/cmd_vel": 4,
	"/local_joy/cmd_vel": 10
}



## Mux priority control class
class NiftiMuxCtrl(object):
	def __init__(self):
		# getting access to the mux
		rospy.loginfo('Waiting for mux service...')
		rospy.wait_for_service('/private/mux_cmd_vel/select')
		rospy.loginfo('Reached mux service.')
		self.mux_select = rospy.ServiceProxy('/private/mux_cmd_vel/select', MuxSelect)

		self.lock = Lock()
		# setting up own services
		self.acquire_srv = rospy.Service('/mux_cmd_vel/acquire', Acquire,
				self.handle_acquire)
		self.release_srv = rospy.Service('/mux_cmd_vel/release', Release,
				self.handle_release)
		# setting initial state
		self.selected = None
		self.requesting = None
		self.stack = ['__none']
		self.mux_select('__none')
		# monitor actual mux state
		rospy.Subscriber('/mux_cmd_vel/selected', String, self.selected_cb)


	def handle_acquire(self, req):
		print self.stack
		resp = False
		self.lock.acquire()
		if priority[self.stack[-1]]<priority.get(req.topic, -1):
			rospy.loginfo('Giving lock to "%s".', req.topic)
			self.requesting = req.topic
			self.mux_select(req.topic)
			self.requesting = None
			if not self.selected == req.topic:
				start = clock()
				sleep(0.001)
				while (not self.selected == req.topic) and (clock()-start<0.5):
					sleep(0.010)
				print "Waited for %f s."%(clock()-start) 
			if self.selected == req.topic:
				resp = True
				rospy.loginfo("Lock given.")
			else:
				rospy.logwarn("Lock failed somehow.")
		self.lock.release()
		print self.stack
		return AcquireResponse(resp)


	def handle_release(self, req):
		print self.stack
		resp = False
		self.lock.acquire()
		if (req.topic in self.stack) and (req.topic != '__none'):
			rospy.loginfo('Releasing request for "%s".', req.topic)
			if self.stack[-1] == req.topic:
				self.stack.pop()
				self.requesting = self.stack[-1]
				self.mux_select(self.requesting)
				self.requesting = None
			else:
				self.stack.remove(req.topic)
			rospy.loginfo("Request released.")
			resp = True
		self.lock.release()
		print self.stack
		return ReleaseResponse(resp)
			
	def selected_cb(self, msg):
		rospy.loginfo('Selected: "%s"', msg.data)
		self.selected = msg.data
		if self.requesting and self.selected!=self.requesting:
			rospy.logerror('Requesting "%s" but selected is "%s"!',
					self.requesting, self.selected)
		if self.stack[-1]!=self.selected:
			self.stack.append(self.selected)



## Create a ROS node and instantiate the class.
def main():
	'''Create a ROS node and instantiate the class.'''
	try:
		rospy.init_node('nifti_mux_control')
		nmc = NiftiMuxCtrl()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	
	
