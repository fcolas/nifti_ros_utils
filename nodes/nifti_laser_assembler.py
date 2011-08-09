#!/usr/bin/env python
## @file
# Laser assembler module for the NIFTi robot
#
# It follows the motion of the laser sensor to trigger the assembly of laser
# scans into a single point cloud.

import roslib; roslib.load_manifest('nifti_laser_assembler')
import rospy

import math
import tf
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud

## Main class to assemble individual laser scans into point clouds
class NiftiLaserAssembler(object):

	## Instantiate NiftiLaserAssembler object
	def __init__(self):
		self.tf_listener = tf.TransformListener()
		self.loop_rate = rospy.Rate(20.0)
		rospy.wait_for_service("assemble_scans")
		self.assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
		self.pointCloud_pub = rospy.Publisher('nifti_pointCloud', PointCloud)


	def spin(self):
		theta_old = 0.0
		start_time = None
		while not rospy.is_shutdown():
			try:
				trans, rot = self.tf_listener.lookupTransform('/laser',
					'/base_link', rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException):
				continue
			#print trans, rot
			# get angle from tf
			theta_raw = 2*math.atan2(rot[0], rot[3]) - math.pi
			# put it back between -pi and pi
			if theta_raw>math.pi:
				theta = theta_raw-2*math.pi
			elif theta_raw<-math.pi:
				theta = theta_raw+2*math.pi
			else:
				theta = theta_raw
			#print theta_raw, theta

			# get start time
			if ((theta <= math.pi/2) and (theta_old > math.pi/2)) or \
					((theta >= -math.pi/2) and (theta_old < -math.pi/2)):
				start_time = rospy.get_rostime()
				rospy.loginfo("start time for assembler")
			# get stop time and publish point cloud
			if start_time and (((theta >= math.pi/2) and (theta_old < math.pi/2)) or \
					((theta <= -math.pi/2) and (theta_old > -math.pi/2))):
				rospy.loginfo("stop time: requesting assembly")
				resp = self.assemble_scans(start_time, rospy.get_rostime())
				rospy.loginfo("publishing cloud")
				self.pointCloud_pub.publish(resp.cloud)
				rospy.loginfo("done")

			theta_old = theta
			self.loop_rate.sleep()


## Create a ROS node and instantiate the NiftiLaserAssembler class.
def main():
	try:
		rospy.init_node('nifti_laser_assembler')
		nla = NiftiLaserAssembler()
		nla.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
