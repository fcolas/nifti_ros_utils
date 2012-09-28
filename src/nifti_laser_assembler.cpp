#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <nifti_pcl_common/point_types.h>
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/transforms.h>
//#include <pcl/point_cloud.h>

//typedef nifti_pcl_common::AssemblerPoint NiftiPoint;
//typedef pcl::PointCloud<NiftiPoint> sensor_msgs::PointCloud2;


template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
	T v;
	if (n.getParam(name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}



/** \brief Class assembling the laser scans of the rolling laser
 *
 * This class manages the assembly of the rolling laser.
 * It listens to the /tf transform of the laser with respect to the robot to
 * trigger the start and end of the assembly and publishes
 * sensor_msgs/PointCloud2 messages.
 */
class NiftiLaserAssembler {
public:
	//! Constructor. ROS::init() is assumed to have been called before.
	NiftiLaserAssembler();

	virtual ~NiftiLaserAssembler();

protected:

	//! /tf listener
	tf::TransformListener tf_listener;

	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	//! Name of the laser frame (default: "/laser")
	std::string laser_frame;

	//! Name of the robot frame (default: "/base_link")
	std::string robot_frame;

	//! Name of the reference world frame (default: "/odom")
	std::string world_frame;

	//! Publisher for the point cloud (default topic: "/static_point_cloud")
	ros::Publisher point_cloud_pub;

	//! Publisher for the moving point cloud (default topic:
	//"/dynamic_point_cloud")
	ros::Publisher dynamic_point_cloud_pub;

	//! Subscriber to input scans (default topic: "/scan_point_cloud")
	ros::Subscriber point_cloud_scan_sub;

	//! Subscriber to input scans (default topic: "/scan_filtered")
	ros::Subscriber laser_scan_sub;

	//! Limit the size of the point cloud in points (default: 1000000).
	int max_size;

	//! Current aggregated point cloud
	sensor_msgs::PointCloud2 point_cloud;

	//! Temporary point cloud
	sensor_msgs::PointCloud2 tmp_point_cloud;

	//! Previous absolute value of the laser angle (to detect when to publish)
	double previous_angle;

	//! Previous absolute value of the laser angle (to detect when to publish)
	double previous_langle;

	//! Starting time of the new scan
	ros::Time start_time;

	//! base_link transform at first time
	tf::StampedTransform base_transform;

	//! flag to invert scan for gmapping
	bool using_gmapping;

	//! Publish 2d scans when laser's horizontal (default: true)
	bool publish2d;

	//! Publisher for the horizontal scans (default topic: "/scan2d")
	ros::Publisher scan2d_pub;
	
	//! Relaying laser scans (default: true)
	bool relay_scans;

	//! Publisher for the relayed scans (default topic: "/scan_relay")
	ros::Publisher relay_pub;

	//! Original scan callback function
	void laserscan_cb(const sensor_msgs::LaserScan& scan);

	//! Coloured scan callback function
	void scan_cb(const sensor_msgs::PointCloud2& scan);

	//! Get the new scan in the point cloud
	void append_scan(const sensor_msgs::PointCloud2& scan);
	
	//! Get laser angle from the tf
	double get_laser_angle(const ros::Time &time) const;

	//! Check if the robot moves
	bool check_no_motion(const ros::Time &time) const;

	//! Subscriber to point cloud control (default topic: "/pointcloud_control")
	ros::Subscriber ptcld_ctrl_sub;

	//! Point cloud control callback
	void ptcld_ctrl_cb(const std_msgs::Bool& on);

	//! Point cloud control current state
	bool ptcld_ctrl_on;
};


/*
 * Constructor
 */
NiftiLaserAssembler::NiftiLaserAssembler():
	tf_listener(ros::Duration(60.)),
	n_("~")
{
	// frame names
	laser_frame = getParam<std::string>(n_, "laser_frame", "/laser");
	robot_frame = getParam<std::string>(n_, "robot_frame", "/base_link");
	world_frame = getParam<std::string>(n_, "world_frame", "/odom");

	// max number of points
	max_size = getParam<int>(n_, "max_size", 1000000);

	using_gmapping = getParam<bool>(n_, "using_gmapping", false);

	// 2d scans
	publish2d = getParam<bool>(n_, "publish2d", true);
	if (publish2d) {
		scan2d_pub = n.advertise<sensor_msgs::LaserScan>("/scan2d", 50);
	}

	// relay
	relay_scans = getParam<bool>(n_, "relay_scans", true);
	if (relay_scans) {
		relay_pub = n.advertise<sensor_msgs::LaserScan>("/scan_relay", 50);
	}

	// moving
	start_time = ros::Time(0);

	// initialized so that the first test always fails
	previous_angle = NAN;
	previous_langle = NAN;

	if (!tf_listener.waitForTransform(laser_frame, world_frame, ros::Time(0),
			ros::Duration(30.)))
	{
		ROS_WARN_STREAM("Timeout (30s) while waiting between "<<laser_frame<<
				" and "<<world_frame<<" at startup.");
	}

	// point cloud publisher
	point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			("/static_point_cloud", 50);
	dynamic_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			("/dynamic_point_cloud", 50);
	
	// laser scan subscriber
	point_cloud_scan_sub = n.subscribe("/scan_point_cloud_color", 50,
			&NiftiLaserAssembler::scan_cb, this);
	if (publish2d||relay_scans) {
		laser_scan_sub = n.subscribe("/scan_filtered", 50,
				&NiftiLaserAssembler::laserscan_cb, this);
	}

	// point cloud control subscriber
	ptcld_ctrl_sub = n.subscribe("/pointcloud_control", 50,
			&NiftiLaserAssembler::ptcld_ctrl_cb, this);
	ptcld_ctrl_on = true;
}


/*
 * Destructor
 */
NiftiLaserAssembler::~NiftiLaserAssembler()
{
	// Nothing to do?
}

/* 
 * Point cloud control callback
 */
void NiftiLaserAssembler::ptcld_ctrl_cb(const std_msgs::Bool& on)
{
	ROS_INFO_STREAM("Point cloud " << (on.data?"enabled.":"disabled."));
	ptcld_ctrl_on = on.data;
}

/*
 * Get the laser angle from the tf_listener
 */
double NiftiLaserAssembler::get_laser_angle(const ros::Time &time) const
{
	double angle;
	tf::StampedTransform tmp_tf;
	geometry_msgs::Quaternion rot;
	if (!tf_listener.waitForTransform(robot_frame, laser_frame, time,
		ros::Duration(1.)))
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and "<<robot_frame<<" before getting laser angle.");
	
	tf_listener.lookupTransform(robot_frame, laser_frame, time, tmp_tf);
	
		
	tf::quaternionTFToMsg(tmp_tf.getRotation(), rot);
	angle = 2.*atan2(rot.x, rot.w) - M_PI;
	if (angle>M_PI)
		angle -= 2.*M_PI;
	else if (angle<-M_PI)
		angle += 2.*M_PI;

	return angle;
}


double norm(const geometry_msgs::Vector3& vec3){
	return sqrt(vec3.x*vec3.x+vec3.y*vec3.y+vec3.z*vec3.z);
}
/*
 * Decide if the robot was still
 */
bool NiftiLaserAssembler::check_no_motion(const ros::Time &time) const
{
	// Checking with velocity but why not position?
	geometry_msgs::Twist mean_speed;
	ros::Duration delta = time - start_time;
	if (delta>=ros::Duration(59.))
		return false;
	if (!tf_listener.waitForTransform(robot_frame, world_frame, time,
		ros::Duration(1.)))
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<robot_frame<<
				" and "<<world_frame<<" before checking motion.");
	tf_listener.lookupTwist(robot_frame, world_frame, start_time+delta*0.5,
			delta, mean_speed);
	ROS_DEBUG_STREAM("Motion: " << norm(mean_speed.linear) << " m/s, " <<
			norm(mean_speed.angular) << " Rad/s for "<<delta.toSec()<<" s"); 
	return ((norm(mean_speed.linear)*delta.toSec()<0.02)&&
			(norm(mean_speed.angular)*delta.toSec()<3*M_PI/180.));
}


/*
 * Invert scan order for gmapping (inverted laser)
 */
void invert_gmapping(const sensor_msgs::LaserScan& scan_in,
			sensor_msgs::LaserScan& scan_out)
{
	unsigned int nb = scan_in.ranges.size();
	for (unsigned int i=0; i<nb; i++)
	{
		scan_out.ranges[i] = scan_in.ranges[nb-i-1];
	}
}

/*
 * Original scan callback function
 */
void NiftiLaserAssembler::laserscan_cb(const sensor_msgs::LaserScan& scan)
{
	double angle;
	try
	{
		angle = get_laser_angle(scan.header.stamp);
	} 
	catch (tf::ExtrapolationException e) {
		ROS_ERROR_STREAM(e.what() << "Could not resolve rotating angle of the laser.");
		return;
	}
	
	if (publish2d) {
		if ((angle*previous_langle<=0.0) ||
				((fabs(angle-previous_langle)<0.5*M_PI/180.)&&
						(fabs(angle)<10*M_PI/180.))) {
			ROS_DEBUG_STREAM("Publishing 2d scan.");
			if (using_gmapping)
			{
				sensor_msgs::LaserScan inv_scan = scan;
				invert_gmapping(scan, inv_scan);
				scan2d_pub.publish(inv_scan);
			} else {
				scan2d_pub.publish(scan);
			}
		}
	}
	if (relay_scans) {
		relay_pub.publish(scan);
	}
	previous_langle = angle;
}

/*
 * laser scan callback
 */
void NiftiLaserAssembler::scan_cb(const sensor_msgs::PointCloud2& scan)
{
	double angle;
	try
	{
		angle = get_laser_angle(scan.header.stamp);
	} 
	catch (tf::ExtrapolationException e) {
		ROS_ERROR_STREAM(e.what() << "Could not resolve rotating angle of the laser.");
		return;
	}
	
	if (fabs(angle)<=M_PI/2) {
		//ROS_INFO_STREAM("Got scan in range.");
		if (start_time.isZero())
			start_time = scan.header.stamp;
		append_scan(scan);

	}

	if ((fabs(previous_angle)<M_PI/2) &&
			(fabs(angle)>=M_PI/2)) {
		//ROS_INFO_STREAM("End");
		if (ptcld_ctrl_on){
			// could decide to update time stamps here (but require a full
			// transform)
			dynamic_point_cloud_pub.publish(point_cloud);
			if (check_no_motion(scan.header.stamp)){
				ROS_DEBUG_STREAM("Publishing static point cloud (" << point_cloud.width << " points).");
				point_cloud_pub.publish(point_cloud);
			} else {
				ROS_DEBUG_STREAM("Point cloud in motion.");
			}
		} else {
			ROS_DEBUG_STREAM("Dropping point cloud (disabled).");
		}
		point_cloud.data.clear();
		point_cloud.width = 0;
		start_time = ros::Time(0);
	}

	// if point cloud is full, we publish it
	// TODO decide if relevant
	if (point_cloud.width>=(unsigned)max_size) {
		ROS_WARN_STREAM("max_size exceeded, clearing.");
		//point_cloud_pub.publish(point_cloud);
		point_cloud.data.clear();
		point_cloud.width = 0;
	}
	previous_angle = angle;
}

/*
 * Append a scan to the current point cloud
 */
void NiftiLaserAssembler::append_scan(const sensor_msgs::PointCloud2& scan)
{
	// Project the scan into the correct space and time reference
	if (point_cloud.width<=0)
	{
		// transform into /base_link
		tf_listener.waitForTransform(robot_frame, scan.header.frame_id,
				scan.header.stamp, ros::Duration(1));
		tf_listener.lookupTransform(robot_frame, scan.header.frame_id,
				scan.header.stamp, base_transform);
		pcl_ros::transformPointCloud(robot_frame, base_transform, scan,
				point_cloud);
		// set base transform for next scans
		tf_listener.lookupTransform(robot_frame, world_frame,
				scan.header.stamp, base_transform);
	} else
	{
		// transformation in /base_link at the original time
		tf::StampedTransform current_transform;
		tf_listener.waitForTransform(world_frame, scan.header.frame_id,
				scan.header.stamp, ros::Duration(1));
		tf_listener.lookupTransform(world_frame, scan.header.frame_id,
				scan.header.stamp, current_transform);
		pcl_ros::transformPointCloud(robot_frame,
				base_transform*current_transform, scan, tmp_point_cloud);
		// fusion of the clouds
		point_cloud.width += tmp_point_cloud.width;
		point_cloud.row_step += tmp_point_cloud.row_step;
		point_cloud.data.insert(point_cloud.data.end(),
				tmp_point_cloud.data.begin(), tmp_point_cloud.data.end());
	}
}


/*
 * Main function
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "nifti_laser_assembler");

	NiftiLaserAssembler nla;

	ros::spin();

	return 0;
}

