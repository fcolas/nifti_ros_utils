#!/usr/bin/env python
## @file
# Set of helper tools for the teleoperation of the NIFTi robot
#
# Right now only ScanningService is implemented: it listens on a topic and
# handles the motion of the rolling laser.
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import PointCloud2
from nifti_robot_driver_msgs.msg import RobotStatus


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
		rospy.Subscriber('/robot_status', RobotStatus, self.status_cb)
		## scanning_once subscriber
		rospy.Subscriber('/scanning_once', Float64, self.scanning_once_cb)
		## point cloud subscriber (to know when to stop)
		rospy.Subscriber('/nifti_point_cloud', PointCloud2, self.point_cloud_cb)
		## scanning state ("Not scanning", "Starting scanning", "Got first cloud")
		self.scanning_state = "Not scanning"

	
	## robot status callback to get the current speed of the laser
	def status_cb(self, robot_status):
		self.scanning_speed = robot_status.scanning_speed
		if self.scanning_speed == 0.0:	# if not moving, updating state
			rospy.logdebug('Laser speed 0: setting state to "Not scanning".')
			self.scanning_state = "Not scanning"


	## scanning once callback to start the laser
	def scanning_once_cb(self, speed):
		if self.scanning_state!="Not scanning": # don't do anything if we're already scanning
			return
		if self.scanning_speed: # don't do anything if the laser is already moving
			return
		# clip speed
		speed = max(min(speed, 0.1), 1.2)
		# send command
		rospy.logdebug("Sending scanning speed command: %f"%speed)
		self.scanning_speed_pub.publish(speed)
		# update state
		self.scanning_state = "Starting scanning"
	

	## point cloud callback to get when to stop the laser
	def point_cloud_cb(self, _):
		if self.scanning_state == "Starting scanning": # no point cloud received yet
			rospy.logdebug("Received first point cloud: waiting for a second one.")
			self.scanning_state = "Got first cloud"
			return
		elif self.scanning_state == "Got first cloud": # a first point cloud received
			rospy.logdebug("Received seconf point cloud: stopping laser and \
centering it.")
			self.scanning_state = "Not scanning"
			self.scanning_speed_pub.publish(0.0) # may be unnecessary
			self.laser_center_pub.publish(True)
			return
		else:
			rospy.logdebug("Point cloud received then ignored.")
			return

def main():
	try:
		# starts the node
		rospy.init_node('nifti_teleop_helper')
		# instantiate the class
		scan3d = ScanningService()
		# wait until closed
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	

