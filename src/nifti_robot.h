#ifndef _NIFTI_ROBOT_H
#define _NIFTI_ROBOT_H

// standard 
#include <string.h>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <librover/librover.h>

#include <nifti_robot_driver_msgs/FlippersState.h>
#include <nifti_robot_driver_msgs/RobotStatus.h>
#include <nifti_robot_driver_msgs/Currents.h>
#include <nifti_robot_driver_msgs/FlipperCommand.h>

//! Maximum scanning speed for the laser
#define MAX_SCANNING_SPEED 1.20


//! Diagnostic messages for the battery
const char BATTERY_DIAG_MSG[][32] = {
	"OK",
	"Warning",
	"Critical"
};


/** \brief Class to handle the NIFTi robot.
 *
 * This class manages the propulsion part of the NIFTi robot: the tracks, the
 * flippers, the differential brake and the rolling mecanism for the laser.
 * 
 * It sends and listens to topics and updates the \c /tf frames.
 * For more information, see \ref interface.
 */
class NiftiRobot {
public:
	//! constructor. ROS::init() must have been called before
	NiftiRobot();
	virtual ~NiftiRobot();

	//! Update all
	void update_all();

	//! Main loop
	void run();

protected:
	// various physical parameters
	//! Lateral distance between center of both tracks
	double robot_width;
	
	//! Height of flippers with respect to tracks
	double flippers_altitude;
	
	//! Length of the tracks
	double tracks_length;
	
	//! Half of the width of both the flippers and the tracks
	double tracks_flippers_half_width;

	//! X coordinate of the laser center with respect to robot
	double laserX;

	//! Y coordinate of the laser center with respect to robot
	double laserY;

	//! Z coordinate of the laser center with respect to robot
	double laserZ;

	//! angle offset of the front left flipper
	double front_left_offset;
	
	//! angle offset of the front right flipper
	double front_right_offset;
	
	//! angle offset of the rear left flipper
	double rear_left_offset;
	
	//! angle offset of the rear right flipper
	double rear_right_offset;
	
	//! maximum track velocity
	double vMax;

	// current state
	//! Current pose
	geometry_msgs::Pose current_pose;
	
	//! Current velocity
	geometry_msgs::Twist current_velocity;
	
	//! Current timestamp
	ros::Time current_timestamp;
	
	//! Physical configuration transforms
	std::vector<geometry_msgs::TransformStamped> configuration_tfs;

	//! Battery state
	int battery_status;

	//! Battery level
	double battery_level;


	// callbacks
	//! Callback for velocity command
	void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel);

	//! Callback for enable command
	void enable_cb(const std_msgs::Bool& on);

	//! Callback for all flippers command
	void flippers_cb(const nifti_robot_driver_msgs::FlippersState& flippers);

	//! Callback for individual flipper command
	void flipper_cb(const nifti_robot_driver_msgs::FlipperCommand& flipperCommand);

	//! Callback for setting scanning speed
	void scanning_speed_cb(const std_msgs::Float64& scanning_speed);

	//! Callback for differential brake
	void brake_cb(const std_msgs::Bool& brake_on);

	//! Callback for laser centering
	void laser_center_cb(const std_msgs::Bool& center);


	// odometry and tfs update
	/**
	 * 2D Motion model
	 * computes linear and angular velocity based on tracks velocity
	 */
	void tracks_to_twist(double vl, double vr, double *v, double *w) const;
	/**
	 * 2D motion model
	 * computes tracks velocity based on linear and angular velocity
	 */
	void twist_to_tracks(double *vl, double *vr, double v, double w) const;
	//geometry_msgs::Twist motion_model_2d(double vl, double vr) const;

	//! Update and publish 2D odometry
	void update_2d_odom();

	//! Update and publish current physical configuration
	void update_config();

	//! Update and publish current robot state
	void update_robot_state();

	
	// ROS stuff
	//! public NodeHandle
	ros::NodeHandle n;
	
	//! Name of the odometry frame
	std::string odom_frame;

	//! Name of the robot frame
	std::string robot_frame;

	//! Name of the laser frame
	std::string laser_frame;

	//! Name of the omnicam frame
	std::string omni_frame;

	//! Name of the imu frame
	std::string imu_frame;


	// publishers
	//! Diagnostics publisher
	diagnostic_updater::Updater diagnostic_pub;

	//! Battery diagnostics
	void diag_batt(diagnostic_updater::DiagnosticStatusWrapper& stat);

	//! 2D odometry publisher in tf
	tf::TransformBroadcaster odom_broadcaster_2d;
	
	//! 2D odometry publisher in message
	ros::Publisher odom_pub;
	
	//! Robot status publisher
	ros::Publisher robot_status_pub;

	//! Configuration publisher
	tf::TransformBroadcaster configuration_broadcaster;

	//! Flippertate publisher
	ros::Publisher flippers_state_pub;

	//! Current readings
	ros::Publisher currents_pub;

	// subscribers
	//! Subscriber to a velocity command
	ros::Subscriber cmd_vel_sub;
	
	//! Subscriber to a enable command
	ros::Subscriber enable_sub;
	
	//! Subscriber to a simulatenous flippers command
	ros::Subscriber flippers_sub;

	//! Subscriber to an individual flipper command
	ros::Subscriber flipper_sub;
	
	//! Subscriber to a command to change the scanning speed
	ros::Subscriber scanning_speed_sub;
	
	//! Subscriber to a differential brake command
	ros::Subscriber brake_sub;

	//! Subscriber to a laser center command
	ros::Subscriber laser_center_sub;


};

#endif
