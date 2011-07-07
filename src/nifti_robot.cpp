#include "nifti_robot.h"

#include <iostream>
#include <math.h>


#define NR_CHECK_AND_RETURN(nrFn, ...) do {if (int e=nrFn(__VA_ARGS__))\
		{ROS_WARN_STREAM("Error " << e << " (" << CAN_error_messages[e]\
			<< ") while calling " << #nrFn <<\
			".");return;}}while (0)

#define GET_BIT(status, bit) (((status) >> (bit)) & 1)

static struct EC_messages EC_messages;


NiftiRobot::NiftiRobot():
	// Lateral distance between center of both tracks
	//robot_width(0.4),
	// Height of flippers with respect to tracks
	//flippers_altitude(0.0195),
	// Length of the tracks
	//tracks_length(0.25),
	// Half of the width of both the flippers and the tracks
	//tracks_flippers_half_width((0.050+0.097)/2.),
	// angle offset of the front left flipper
	//front_left_offset(11.1*M_PI/180.0),
	// angle offset of the front right flipper
	//front_right_offset(11.1*M_PI/180.0),
	// angle offset of the rear left flipper
	//rear_left_offset((180-11.1)*M_PI/180.),
	//? angle offset of the rear right flipper
	//rear_right_offset((180-11.1)*M_PI/180.),
	// Battery status
	battery_status(-1),
	// Battery level
	battery_level(-1),
	// Name of the odometry frame
	odom_frame("/odom"),
	// Name of the robot frame
	robot_frame("/base_link"),
	// Name of the laser frame
	laser_frame("/laser"),
	// Name of the omnicam frame
	omni_frame("/omni"),
	// Name of the imu frame
	imu_frame("/imu"),
	// diagnostics publisher
	//diagnostic_pub(), // Doesn't work!?
	// 2D odometry publisher in tf
	odom_broadcaster_2d(),
	// 2D odometry publisher in message
	odom_pub(n.advertise<nav_msgs::Odometry>("odom", 50)),
	// Robot status publisher
	robot_status_pub(n.advertise<nifti_robot_driver_msgs::RobotStatus>
			("robot_status", 50)),
	// Configuration publisher
	configuration_broadcaster(),
	flippers_state_pub(n.advertise<nifti_robot_driver_msgs::FlippersState>
			("flippers_state", 50)),
	currents_pub(n.advertise<nifti_robot_driver_msgs::Currents>
			("currents", 50))
	// subscribers are initialized at the end, after nrInit
{

	/*
	 * initialize drivers library
	 */
	std::string CAN_device;
	n.param<std::string>("CAN_device", CAN_device, "/dev/usb/cpc_usb0");
	RoverParams params;
	nrGetDefaultParams(&params);
	char c_CAN_device[CAN_device.size()+1];
	std::strncpy(c_CAN_device, CAN_device.c_str(), CAN_device.size()+1);
	bool bestInit;
	n.param<bool>("bestInit", bestInit, true);
	ROS_INFO_STREAM("trying to " << (bestInit?"best ":"") << "init " << c_CAN_device);
	nrInit(c_CAN_device, &params, bestInit);

	robot_width = params.trackDistance;
	flippers_altitude = params.trackWheelRadius - params.referentialZ;
	tracks_length = params.trackLength;
	tracks_flippers_half_width = (params.trackWidth+params.flipperWidth)/2.0;
	front_left_offset = params.flipperOffset;
	front_right_offset = params.flipperOffset;
	rear_left_offset = M_PI - params.flipperOffset;
	rear_right_offset = M_PI - params.flipperOffset;
	vMax = params.vMax;
	laserX = params.laserX;
	laserY = params.laserY;
	laserZ = params.laserZ;
	// TODO get robot parameters from the library

	// 2D odometry initialization
	current_pose.position.x = 0.0;
	current_pose.position.y = 0.0;
	current_pose.position.z = 0.0;
	current_pose.orientation.x = 0.0;
	current_pose.orientation.y = 0.0;
	current_pose.orientation.z = 0.0;
	current_pose.orientation.w = 1.0;
	current_velocity.linear.x = 0.0;
	current_velocity.linear.y = 0.0;
	current_velocity.linear.z = 0.0;
	current_velocity.angular.x = 0.0;
	current_velocity.angular.y = 0.0;
	current_velocity.angular.z = 0.0;
	current_timestamp=ros::Time::now();
	n.param<std::string>("odom_frame", odom_frame, "odom");
	n.param<std::string>("robot_frame", robot_frame, "base_link");
	n.param<std::string>("laser_frame", laser_frame, "laser");
	n.param<std::string>("omni_frame", omni_frame, "omni");
	n.param<std::string>("imu_frame", imu_frame, "imu");

	// configuration tf
	geometry_msgs::TransformStamped left_track_tf;
	left_track_tf.header.frame_id = robot_frame;
	left_track_tf.child_frame_id = "left_track";
	configuration_tfs.push_back(left_track_tf);
	geometry_msgs::TransformStamped right_track_tf;
	right_track_tf.header.frame_id = robot_frame;
	right_track_tf.child_frame_id = "right_track";
	configuration_tfs.push_back(right_track_tf);
	geometry_msgs::TransformStamped front_left_flipper_tf;
	front_left_flipper_tf.header.frame_id = "left_track";
	front_left_flipper_tf.child_frame_id = "front_left_flipper";
	configuration_tfs.push_back(front_left_flipper_tf);
	geometry_msgs::TransformStamped rear_left_flipper_tf;
	rear_left_flipper_tf.header.frame_id = "left_track";
	rear_left_flipper_tf.child_frame_id = "rear_left_flipper";
	configuration_tfs.push_back(rear_left_flipper_tf);
	geometry_msgs::TransformStamped front_right_flipper_tf;
	front_right_flipper_tf.header.frame_id = "right_track";
	front_right_flipper_tf.child_frame_id = "front_right_flipper";
	configuration_tfs.push_back(front_right_flipper_tf);
	geometry_msgs::TransformStamped rear_right_flipper_tf;
	rear_right_flipper_tf.header.frame_id = "right_track";
	rear_right_flipper_tf.child_frame_id = "rear_right_flipper";
	configuration_tfs.push_back(rear_right_flipper_tf);
	geometry_msgs::TransformStamped laser_tf;
	laser_tf.header.frame_id = robot_frame;
	laser_tf.child_frame_id = laser_frame;
	configuration_tfs.push_back(laser_tf);

	// fixed frames:
	// omnicam
	geometry_msgs::Quaternion tmp_rot;
	tmp_rot.x = 0.;
	tmp_rot.y = 0.;
	tmp_rot.z = sin(params.omniAngleOffset/2.);
	tmp_rot.w = cos(params.omniAngleOffset/2.);
	geometry_msgs::TransformStamped omni_tf;
	omni_tf.header.frame_id = robot_frame;
	omni_tf.child_frame_id = omni_frame;
	omni_tf.transform.translation.x = params.omniX;
	omni_tf.transform.translation.y = params.omniY;
	omni_tf.transform.translation.z = params.omniZ;
	omni_tf.transform.rotation = tmp_rot;
	configuration_tfs.push_back(omni_tf);
	// IMU
	geometry_msgs::TransformStamped imu_tf;
	imu_tf.header.frame_id = robot_frame;
	imu_tf.child_frame_id = imu_frame;
	imu_tf.transform.translation.x = params.imuX;
	imu_tf.transform.translation.y = params.imuY;
	imu_tf.transform.translation.z = params.imuZ;
	tmp_rot.z = 0.;
	tmp_rot.w = 1.;
	imu_tf.transform.rotation = tmp_rot;
	configuration_tfs.push_back(imu_tf);

	// setting up diagnostics
	//diagnostic_updater::Updater tmp_up;
	//diagnostic_pub = tmp_up;
    diagnostic_pub.setHardwareID("none");
    diagnostic_pub.add("Battery", this, &NiftiRobot::diag_batt);
    diagnostic_pub.add("Core", this, &NiftiRobot::diag_core);
    diagnostic_pub.add("Left track", this, &NiftiRobot::diag_left_track);
    diagnostic_pub.add("Right track", this, &NiftiRobot::diag_right_track);
    diagnostic_pub.add("Front left flipper", this, &NiftiRobot::diag_front_left_flipper);
    diagnostic_pub.add("Front right flipper", this, &NiftiRobot::diag_front_right_flipper);
    diagnostic_pub.add("Rear left flipper", this, &NiftiRobot::diag_rear_left_flipper);
    diagnostic_pub.add("Rear right flipper", this, &NiftiRobot::diag_rear_right_flipper);

	

	/*
	 * initialize and subscribe the listeners
	 * could be done at construction 
	 * but here to make sure that nrInit is called before any callback 
	 */
	// velocity command
	cmd_vel_sub = n.subscribe("cmd_vel", 1, &NiftiRobot::cmd_vel_cb, this);
	// enable command
	enable_sub = n.subscribe("enable", 1, &NiftiRobot::enable_cb, this);
	// all flippers command
	flippers_sub = n.subscribe("flippers_cmd", 1, &NiftiRobot::flippers_cb, this);
	// individual flipper command
	flipper_sub = n.subscribe("flipper_cmd", 4, &NiftiRobot::flipper_cb, this);
	// scanning speed command
	scanning_speed_sub = n.subscribe("scanning_speed_cmd", 1, &NiftiRobot::scanning_speed_cb, this);
	// brake command
	brake_sub = n.subscribe("brake", 1, &NiftiRobot::brake_cb, this);
	// laser centering command
	laser_center_sub = n.subscribe("laser_center", 1,
		&NiftiRobot::laser_center_cb, this);
}

/*
 * destructor
 */
NiftiRobot::~NiftiRobot()
{
	// cleanup
	nrDestroy();
}

/*
 * Callback for velocity command
 */
#define EPSILON 0.00001
void NiftiRobot::cmd_vel_cb(const geometry_msgs::Twist& cmd_vel)
{
	ROS_DEBUG_STREAM("received velocity command: " << cmd_vel);
	double vr, vl;
	twist_to_tracks(&vl, &vr, cmd_vel.linear.x, cmd_vel.angular.z);
	if ((vl<=(vMax+EPSILON)) && (vl>=-(vMax+EPSILON)) && (vr<=(vMax+EPSILON)) &&
			(vr>=-(vMax+EPSILON)))
		NR_CHECK_AND_RETURN(nrSetSpeed, cmd_vel.linear.x, cmd_vel.angular.z);
	else
		ROS_WARN_STREAM("Invalid velocity command (v="<<cmd_vel.linear.x\
				<<", w="<<cmd_vel.angular.z<<") -> (vr="<<vr<<\
				", vl="<<vl<<")."<<vr-0.6<<" "<<vl-0.6);
}

/*
 * Callback for enable command
 */
void NiftiRobot::enable_cb(const std_msgs::Bool& on)
{
	ROS_INFO_STREAM("received enable command: " << (on.data?"True":"False"));
	if (on.data)
		NR_CHECK_AND_RETURN(nrEnable, 1);
	else
		NR_CHECK_AND_RETURN(nrEnable, 0);
}


/*
 * Callback for all flippers command
 */
void NiftiRobot::flippers_cb(const nifti_robot_driver_msgs::FlippersState& flippers)
{
	ROS_DEBUG_STREAM("received flippers command: " << flippers);
	NR_CHECK_AND_RETURN(nrSetFlippers, flippers.frontLeft, flippers.frontRight,
			flippers.rearLeft, flippers.rearRight);
}

/*
 * Callback for individual flipper command
 */
void NiftiRobot::flipper_cb(const nifti_robot_driver_msgs::FlipperCommand& flipperCommand)
{
	ROS_DEBUG_STREAM("received individual flipper command: " << flipperCommand);
	NR_CHECK_AND_RETURN(nrSetFlipper, flipperCommand.angle, flipperCommand.object_id);
}

/*
 * Callback for setting scanning speed
 */
void NiftiRobot::scanning_speed_cb(const std_msgs::Float64& scanning_speed)
{
	ROS_DEBUG_STREAM("received scanning speed command: " << scanning_speed.data);
	if ((scanning_speed.data<=MAX_SCANNING_SPEED) && (scanning_speed.data>=0.0))
		NR_CHECK_AND_RETURN(nrSetScanningSpeed, scanning_speed.data);
	else
		ROS_WARN_STREAM("Invalid scanning speed: "<<scanning_speed.data);
}

/*
 * Callback for differential brake
 */
void NiftiRobot::brake_cb(const std_msgs::Bool& brake_on)
{
	ROS_DEBUG_STREAM("received brake command: " << brake_on.data);
	NR_CHECK_AND_RETURN(nrSetBrake, brake_on.data);
}

/*
 * Callback for laser centering command
 */
void NiftiRobot::laser_center_cb(const std_msgs::Bool& center)
{
	ROS_DEBUG_STREAM("received laser center command: " << center.data);
	NR_CHECK_AND_RETURN(nrGoMiddlePos);
}

/*
 * 2D Motion model
 * computes linear and angular velocity based on tracks velocity
 */
void NiftiRobot::tracks_to_twist(double vl, double vr, double *v, double *w) const
{
	*v = (vl+vr)/2.0;
	*w = (vr-vl)/robot_width;
}

/*
 * 2D motion model
 * computes tracks velocity based on linear and angular velocity
 */
void NiftiRobot::twist_to_tracks(double *vl, double *vr, double v, double w) const
{
	*vr = v + w*robot_width/2.0;
	*vl = v - w*robot_width/2.0;
}

/*
 * 2D Motion model
 * computes velocity based on each track speed
 */
/*
geometry_msgs::Twist NiftiRobot::motion_model_2d(double vl, double vr) const
{
	geometry_msgs::Twist velocity;
	velocity.linear.x = (vl+vr)/2.0;
	velocity.linear.y = 0.0;
	velocity.linear.z = 0.0;
	velocity.angular.x = 0.0;
	velocity.angular.y = 0.0;
	velocity.angular.z = (vr-vl)/robot_width;
	return velocity;
}
*/

/*
 * update and publish 2D odometry
 */
void NiftiRobot::update_2d_odom()
{
	// get current velocity
	double v, w;
	NR_CHECK_AND_RETURN(nrGetSpeed, &v, &w);
	ros::Time new_timestamp = ros::Time::now();
	geometry_msgs::Twist new_velocity;
	new_velocity.linear.x = v;
	new_velocity.linear.y = 0.0;
	new_velocity.linear.z = 0.0;
	new_velocity.angular.x = 0.0;
	new_velocity.angular.y = 0.0;
	new_velocity.angular.z = w;

	
	// update pose
	double dt, d, dtheta, dx, dy, ctheta, stheta, ctheta2, stheta2;
	// displacement in old reference frame
	dt = (new_timestamp - current_timestamp).toSec();
	d = dt * (new_velocity.linear.x + current_velocity.linear.x)/2.0;	// constant acceleration 
	dtheta = dt * (new_velocity.angular.z + current_velocity.angular.z)/2.0;
	if (abs(dtheta)<0.0001) { // straight path
		dx = d;
		dy = 0.0;
	} else {	// circular approximation
		dx = d/dtheta * sin(dtheta);
		dy = d/dtheta * (1 - cos(dtheta));
	}
	// rotation of the displacement by the orientation
	ctheta2 = current_pose.orientation.w;
	stheta2 = current_pose.orientation.z;
	ctheta = ctheta2*ctheta2 - stheta2*stheta2;
	stheta = 2*ctheta2*stheta2;
	current_pose.position.x += ctheta * dx - stheta * dy;
	current_pose.position.y += stheta * dx + ctheta * dy;
	// update of the orientation
	current_pose.orientation.w = ctheta2 * cos(dtheta/2.0) - stheta2 * sin(dtheta/2.0);
	current_pose.orientation.z = ctheta2 * sin(dtheta/2.0) + stheta2 * cos(dtheta/2.0);
	
	// update velocity and timestamp
	current_velocity = new_velocity;
	current_timestamp = new_timestamp;

	// publish odometry in tf
    geometry_msgs::TransformStamped odom_trans_2d;
	odom_trans_2d.header.frame_id = odom_frame;
	odom_trans_2d.child_frame_id = robot_frame;
	odom_trans_2d.header.stamp = current_timestamp;
	odom_trans_2d.transform.translation.x = current_pose.position.x;
	odom_trans_2d.transform.translation.y = current_pose.position.y;
	odom_trans_2d.transform.translation.z = current_pose.position.z;
	odom_trans_2d.transform.rotation = current_pose.orientation;
	odom_broadcaster_2d.sendTransform(odom_trans_2d);

	// publish odometry with Odom message
	nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_timestamp;
    odom_msg.header.frame_id = odom_frame;
    //set the position
    odom_msg.pose.pose = current_pose;
    //set the velocity
    odom_msg.child_frame_id = robot_frame;
    odom_msg.twist.twist = current_velocity;
	odom_pub.publish(odom_msg);
	
}


/*
 * Update and publish current physical configuration
 */
void NiftiRobot::update_config()
{
	double left_angle, right_angle;
	double frontLeft, frontRight, rearLeft, rearRight;
	double laser_angle;
	NR_CHECK_AND_RETURN(nrGetDifferentialAngles, &left_angle, &right_angle);
	ROS_DEBUG_STREAM("angles: "<< left_angle << "Rad and " << right_angle << "Rad");
	NR_CHECK_AND_RETURN(nrGetFlippers, &frontLeft, &frontRight, &rearLeft, &rearRight);
	NR_CHECK_AND_RETURN(nrGetScannerAngle, &laser_angle);
	ros::Time timestamp = ros::Time::now();

	nifti_robot_driver_msgs::FlippersState flippers_state_msg;
	flippers_state_msg.frontLeft = frontLeft;
	flippers_state_msg.frontRight = frontRight;
	flippers_state_msg.rearLeft = rearLeft;
	flippers_state_msg.rearRight = rearRight;

	// left track
	configuration_tfs[0].header.stamp = timestamp;
	configuration_tfs[0].transform.translation.x = 0.0;
	configuration_tfs[0].transform.translation.y = robot_width/2.0;
	configuration_tfs[0].transform.translation.z = 0.0;
	configuration_tfs[0].transform.rotation.x = 0.0;
	configuration_tfs[0].transform.rotation.y = sin(left_angle/2.);
	configuration_tfs[0].transform.rotation.z = 0.0;
	configuration_tfs[0].transform.rotation.w = cos(left_angle/2.);
	// right track
	configuration_tfs[1].header.stamp = timestamp;
	configuration_tfs[1].transform.translation.x = 0.0;
	configuration_tfs[1].transform.translation.y = -robot_width/2.0;
	configuration_tfs[1].transform.translation.z = 0.0;
	configuration_tfs[1].transform.rotation.x = 0.0;
	configuration_tfs[1].transform.rotation.y = sin(right_angle/2.);
	configuration_tfs[1].transform.rotation.z = 0.0;
	configuration_tfs[1].transform.rotation.w = cos(right_angle/2.);
	// front left flipper
	configuration_tfs[2].header.stamp = timestamp;
	configuration_tfs[2].transform.translation.x = tracks_length/2.;
	configuration_tfs[2].transform.translation.y = tracks_flippers_half_width;
	configuration_tfs[2].transform.translation.z = flippers_altitude;
	configuration_tfs[2].transform.rotation.x = 0.0;
	configuration_tfs[2].transform.rotation.y = sin((frontLeft+front_left_offset)/2.);
	configuration_tfs[2].transform.rotation.z = 0.0;
	configuration_tfs[2].transform.rotation.w = cos((frontLeft+front_left_offset)/2.);
	// rear left flipper
	configuration_tfs[3].header.stamp = timestamp;
	configuration_tfs[3].transform.translation.x = -tracks_length/2.;
	configuration_tfs[3].transform.translation.y = tracks_flippers_half_width;
	configuration_tfs[3].transform.translation.z = flippers_altitude;
	configuration_tfs[3].transform.rotation.x = 0.0;
	configuration_tfs[3].transform.rotation.y = sin((rearLeft+rear_left_offset)/2.);
	configuration_tfs[3].transform.rotation.z = 0.0;
	configuration_tfs[3].transform.rotation.w = cos((rearLeft+rear_left_offset)/2.);
	// front right flipper
	configuration_tfs[4].header.stamp = timestamp;
	configuration_tfs[4].transform.translation.x = tracks_length/2.;
	configuration_tfs[4].transform.translation.y = -tracks_flippers_half_width;
	configuration_tfs[4].transform.translation.z = flippers_altitude;
	configuration_tfs[4].transform.rotation.x = 0.0;
	configuration_tfs[4].transform.rotation.y = sin((frontRight+front_right_offset)/2.);
	configuration_tfs[4].transform.rotation.z = 0.0;
	configuration_tfs[4].transform.rotation.w = cos((frontRight+front_right_offset)/2.);
	// rear right flipper
	configuration_tfs[5].header.stamp = timestamp;
	configuration_tfs[5].transform.translation.x = -tracks_length/2.;
	configuration_tfs[5].transform.translation.y = -tracks_flippers_half_width;
	configuration_tfs[5].transform.translation.z = flippers_altitude;
	configuration_tfs[5].transform.rotation.x = 0.0;
	configuration_tfs[5].transform.rotation.y = sin((rearRight+rear_right_offset)/2.);
	configuration_tfs[5].transform.rotation.z = 0.0;
	configuration_tfs[5].transform.rotation.w = cos((rearRight+rear_right_offset)/2.);
	// laser
	configuration_tfs[6].header.stamp = timestamp;
	configuration_tfs[6].transform.translation.x = laserX;
	configuration_tfs[6].transform.translation.y = laserY;
	configuration_tfs[6].transform.translation.z = laserZ;
	configuration_tfs[6].transform.rotation.x = sin((M_PI+laser_angle)/2.);
	configuration_tfs[6].transform.rotation.y = 0.0;
	configuration_tfs[6].transform.rotation.z = 0.0;
	configuration_tfs[6].transform.rotation.w = cos((M_PI+laser_angle)/2.);
	// update timestamp for omnicam and IMU
	configuration_tfs[7].header.stamp = timestamp;
	configuration_tfs[8].header.stamp = timestamp;
	// publish configuration
	configuration_broadcaster.sendTransform(configuration_tfs);

	// publish the flippers status
	flippers_state_pub.publish(flippers_state_msg);
}

/*
 * Update and publish current robot state
 */
void NiftiRobot::update_robot_state()
{
	//int controllers_status[7];
	int brake_on;
	int err = 0;
	double scanning_speed;
	nifti_robot_driver_msgs::RobotStatus robot_status;
	nifti_robot_driver_msgs::Currents currents;
	double current;


	battery_level = -1.0;
	NR_CHECK_AND_RETURN(nrGetBatteryLevel, &battery_level, &battery_status);
	NR_CHECK_AND_RETURN(nrGetControllersStatus, controllers_status);
	for (int i=0; i<ID_CTRL_MAX; err+=SR_GET_ERROR(controllers_status[i++]));
	if (err)
		NR_CHECK_AND_RETURN(nrGetControllersError, controllers_error);
	else
		for (int i=0; i<ID_CTRL_MAX; controllers_error[i++]=0);
	NR_CHECK_AND_RETURN(nrGetBrake, &brake_on);
	NR_CHECK_AND_RETURN(nrGetScanningSpeed, &scanning_speed);
	//ROS_INFO_STREAM("Scanning_speed: " << scanning_speed);

	robot_status.battery_level = battery_level;
	robot_status.battery_status = battery_status;
	robot_status.brake_on = brake_on;
	robot_status.scanning_speed = scanning_speed;
	robot_status.controllers_status.core = controllers_status[ID_CORE];
	robot_status.controllers_status.track_left =
			controllers_status[ID_TRACK_LEFT];
	robot_status.controllers_status.track_right =
			controllers_status[ID_TRACK_RIGHT];
	robot_status.controllers_status.flipper_front_left =
			controllers_status[ID_FLIPPER_FRONT_LEFT];
	robot_status.controllers_status.flipper_front_right =
			controllers_status[ID_FLIPPER_FRONT_RIGHT];
	robot_status.controllers_status.flipper_rear_left =
			controllers_status[ID_FLIPPER_REAR_LEFT];
	robot_status.controllers_status.flipper_rear_right =
			controllers_status[ID_FLIPPER_REAR_RIGHT];
	robot_status.controllers_error.core = controllers_error[ID_CORE];
	robot_status.controllers_error.track_left =
			controllers_error[ID_TRACK_LEFT];
	robot_status.controllers_error.track_right =
			controllers_error[ID_TRACK_RIGHT];
	robot_status.controllers_error.flipper_front_left =
			controllers_error[ID_FLIPPER_FRONT_LEFT];
	robot_status.controllers_error.flipper_front_right =
			controllers_error[ID_FLIPPER_FRONT_RIGHT];
	robot_status.controllers_error.flipper_rear_left =
			controllers_error[ID_FLIPPER_REAR_LEFT];
	robot_status.controllers_error.flipper_rear_right =
			controllers_error[ID_FLIPPER_REAR_RIGHT];
	
	robot_status_pub.publish(robot_status);

	NR_CHECK_AND_RETURN(nrReadDOFCurrent, &current, ID_TRACK_LEFT);
	currents.trackLeft=current;
	NR_CHECK_AND_RETURN(nrReadDOFCurrent, &current, ID_TRACK_RIGHT);
	currents.trackRight=current;
	NR_CHECK_AND_RETURN(nrReadDOFCurrent, &current, ID_FLIPPER_FRONT_LEFT);
	currents.frontLeft=current;
	NR_CHECK_AND_RETURN(nrReadDOFCurrent, &current, ID_FLIPPER_FRONT_RIGHT);
	currents.frontRight=current;
	NR_CHECK_AND_RETURN(nrReadDOFCurrent, &current, ID_FLIPPER_REAR_LEFT);
	currents.rearLeft=current;
	NR_CHECK_AND_RETURN(nrReadDOFCurrent, &current, ID_FLIPPER_REAR_RIGHT);
	currents.rearRight=current;
	
	currents_pub.publish(currents);

}

/*
 * Battery diagnostics
 */
void NiftiRobot::diag_batt(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
	if (battery_status>-1)
		stat.summary(battery_status, battery_messages[battery_status]);
	else
		stat.summary(2, "No battery information.");
	
	stat.add("battery level", battery_level);
    
}

/*
 * Controllers diagnostics
 */
void sprintf_binary8(char* buffer, char value) {
	sprintf(buffer, "%s%s%s%s.%s%s%s%s",
		(GET_BIT(value, 7)?"1":"0"),
		(GET_BIT(value, 6)?"1":"0"),
		(GET_BIT(value, 5)?"1":"0"),
		(GET_BIT(value, 4)?"1":"0"),
		(GET_BIT(value, 3)?"1":"0"),
		(GET_BIT(value, 2)?"1":"0"),
		(GET_BIT(value, 1)?"1":"0"),
		(GET_BIT(value, 0)?"1":"0"));
}

void sprintf_binary32(char* buffer, int value) {
	char buf3[10];
	char buf2[10];
	char buf1[10];
	char buf0[10];
	sprintf_binary8(buf3, (value&0xff000000)>>24);
	sprintf_binary8(buf2, (value&0x00ff0000)>>16);
	sprintf_binary8(buf1, (value&0x0000ff00)>>8);
	sprintf_binary8(buf0, (value&0x000000ff)>>0);
	sprintf(buffer, "%s:%s:%s:%s", buf3, buf2, buf1, buf0);

}


void diag_ctrl(diagnostic_updater::DiagnosticStatusWrapper& stat, int status,
		int error)
{
	if (SR_GET_ERROR(status)) {
		stat.summary(2, "Error: "+EC_messages.get(error));
		stat.add("Error code (EC)", EC_messages.get(error));
	} else if (!SR_GET_MOTOR_ON(status))
		stat.summary(1, "Motor disabled");
	else
		stat.summary(0, "OK");
	stat.add("Error flag", SR_GET_ERROR(status));
	stat.add("Servo drive status",
			servo_drive_status_messages[SR_GET_SERVO_DRIVE_STATUS(status)]);
	stat.add("Motor on (MO)", MO_messages[SR_GET_MOTOR_ON(status)]);
	stat.add("Unit mode (UM)", UM_messages[(status&0x0380)>>7]);
//	stat.add("Gain scheduling on", GET_BIT(status, 10));
//	stat.add("Program running", GET_BIT(status, 12));
//	stat.add("Motion status (MS)", MS_messages[(status&0x0C00)>>14]);
//	stat.add("Stopped by a limit", GET_BIT(status, 28));
	stat.add("Error in user program",  GET_BIT(status, 29));
	char buffer[40];
	sprintf_binary32(buffer, status);
	stat.add("Status register (SR)", buffer);
//	char stat_name[10];
//	for (int i=0;i<32;i++) {
//		sprintf(stat_name, "Bit %d", i);
//		stat.add(stat_name, GET_BIT(status, i));
//	}
}

void NiftiRobot::diag_core(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	diag_ctrl(stat, controllers_status[ID_CORE], controllers_error[ID_CORE]);
}
void NiftiRobot::diag_left_track(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	diag_ctrl(stat, controllers_status[ID_TRACK_LEFT], controllers_error[ID_TRACK_LEFT]);
}
void NiftiRobot::diag_right_track(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	diag_ctrl(stat, controllers_status[ID_TRACK_RIGHT], controllers_error[ID_TRACK_RIGHT]);
}
void NiftiRobot::diag_front_left_flipper(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	diag_ctrl(stat, controllers_status[ID_FLIPPER_FRONT_LEFT], controllers_error[ID_FLIPPER_FRONT_LEFT]);
}
void NiftiRobot::diag_front_right_flipper(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	diag_ctrl(stat, controllers_status[ID_FLIPPER_FRONT_RIGHT], controllers_error[ID_FLIPPER_FRONT_RIGHT]);
}
void NiftiRobot::diag_rear_left_flipper(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	diag_ctrl(stat, controllers_status[ID_FLIPPER_REAR_LEFT], controllers_error[ID_FLIPPER_REAR_LEFT]);
}
void NiftiRobot::diag_rear_right_flipper(diagnostic_updater::DiagnosticStatusWrapper& stat) {
	diag_ctrl(stat, controllers_status[ID_FLIPPER_REAR_RIGHT], controllers_error[ID_FLIPPER_REAR_RIGHT]);
}

/*
 * Update all
 */
void NiftiRobot::update_all()
{
	update_robot_state();
	update_config();
	update_2d_odom();
	diagnostic_pub.update();
}


/*
 * Main loop
 */
void NiftiRobot::run(){

	// getting frequency
	double frequency;
	n.param<double>("loop_rate", frequency, 20.0);
	ros::Rate loop_rate(frequency);
	ROS_DEBUG("Looping at %fHz.", frequency);

	// loop
	while (ros::ok())
	{
		update_all();
		loop_rate.sleep();
		ros::spinOnce();
	}
	NR_CHECK_AND_RETURN(nrEnable, 0);

}


