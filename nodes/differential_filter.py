#!/usr/bin/env python
import roslib; roslib.load_manifest('nifti_robot_driver')
import rospy
from sensor_msgs.msg import JointState
from math import radians, degrees, sqrt

class DifferentialFilter(object):
	def __init__(self, N, s0):
		self.N = N
		self.s0 = s0
		self.bias_left = self.bias_right = 0
		self.K = 1
		self.last_seq = None
		self.last_left = self.last_right = None
		self.acc = 0
		self.joints_pub = rospy.Publisher("/filtered_joint_states", JointState)
		rospy.Subscriber("/joint_states", JointState, self.js_cb)
	
	def js_cb(self, joints):
		left = joints.position[joints.name.index("left_track_j")]
		right = joints.position[joints.name.index("right_track_j")]
		seq = joints.header.seq
		K = self.K
		if not self.last_seq:
			self.last_seq = seq+self.N
		if seq>self.last_seq:
			n_left = (1-K)*self.last_left + K*(left-self.bias_left)
			new_pos = list(joints.position)
			new_pos[joints.name.index("left_track_j")] = n_left
			n_right = (1-K)*self.last_right + K*(right-self.bias_right)
			new_pos[joints.name.index("right_track_j")] = n_right
			joints.position = new_pos
		else:
			self.bias_left += left/self.N
			self.acc += left*left/self.N/2
			self.last_left = left
			self.bias_right += right/self.N
			self.acc += right*right/self.N/2
			self.last_right = right
			bl = self.bias_left
			br = self.bias_right
			s = self.s0
			self.K = s/(s+sqrt(self.acc-0.5*(bl*bl+br*br)))
			n_left = left
			n_right = right
		if seq==self.last_seq:
			rospy.loginfo("left bias=%f deg, right bias=%f deg, K=%f",
					degrees(self.bias_left), degrees(self.bias_right), self.K)
		self.joints_pub.publish(joints)

## Create a ROS node and instantiate the class.
def main():
	'''Create a ROS node and instantiate the class.'''
	try:
		rospy.init_node('differential_filter')
		df = DifferentialFilter(N=150, s0=radians(0.5))
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__== '__main__':
	main()
