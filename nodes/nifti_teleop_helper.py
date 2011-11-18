#!/usr/bin/env python

import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from std_msgs.msg import Bool, Float64, PointCloud2
from nifti_robot_driver_msgs.msg import RobotStatus


class ScanningService(object):
	def __init__(self):
		self.scanning_speed_pub = rospy.Publisher('/scanning_speed_cmd', Float64)
		self.laser_center_pub = rospy.Publisher('/laser_center', Bool)
		rospy.Subscriber('/robot_status', RobotStatus, self.status_cb)
		rospy.Subscriber('/scanning_once', Float64, self.scanning_once_cb)
		rospy.Subscriber('/nifti_point_cloud', PointCloud2, self.point_cloud_cb)
		self.scanning_state = "Not scanning"

	def status_cb(self, robot_status):
		self.scanning_speed = robot_status.scanning_speed

	def scanning_once_cb(self, speed):
		if self.scanning!="Not scanning": # don't do anything if we're already scanning
			return
		if self.scanning_speed: # don't do anything if the laser is already moving
			return
		self.scanning_state = "Starting scanning"
		speed = max(min(speed, 0.1), 1.2)
		self.scanning_speed_pub.publish(speed)
	
	def point_cloud_cb(self, _):
		if self.scanning_state == "Starting scanning":
			self.scanning_state = "Got first cloud"
			return
		elif self.scanning_state == "Got first cloud":
			self.scanning_state = "Not scanning"
			self.laser_center_pub.publish(True)
			return
		else:
			return

def main():
	try:
		rospy.init_node('nifti_teleop_helper')
		scan3d = ScanningService()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	

