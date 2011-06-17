
#include <iostream>
#include <ros/ros.h>
#include "nifti_robot.h"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "nifti_ros_drivers");

	// must be done after ROS init
	NiftiRobot robot = NiftiRobot();

	robot.run();

	return 0;

}
