
#include <math.h>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <laser_filters/scan_shadows_filter.h>



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

	//! Publisher for the point cloud (default topic: "/nifti_point_cloud")
	ros::Publisher point_cloud_pub;

	//! Subscriber to laser scans (default topic: "/scan")
	ros::Subscriber laser_scan_sub;

	//! Limit the size of the point cloud in points (default: 1000000).
	int max_size;

	//! Current aggregated point cloud
	sensor_msgs::PointCloud2 point_cloud;

	//! Scan to filter and modify
	sensor_msgs::LaserScan tmp_scan;

	//! Point cloud holding the projection of one laser scan
	sensor_msgs::PointCloud2 tmp_point_cloud;

	//! Previous absolute value of the laser angle (to detect when to publish)
	double previous_angle;

	//! Laser angle offset (we publish slightly slanted when not moving
	double laser_angle_offset;

	//! Projector object from LaserScan to PointCloud2
	laser_geometry::LaserProjection projector;

	//! Flag allowing pointcloud publication if the robot moved (default: true)
	bool publish_in_motion;

	//! Starting time of the new scan
	ros::Time start_time;

	/** \brief Set of channel to add to the point cloud (default: none)
	 *
	 * This follows the laser_geometry::channel_option::ChannelOption enum.
	 * By default, we don't add any channel. Parameter ~channels can be set to
	 * change that.
	 */
	int point_cloud_channels;

	//! Publish 2d scans when laser's horizontal (default: true)
	bool publish2d;

	//! Publisher for the horizontal scans (default topic: "/scan2d")
	ros::Publisher scan2d_pub;
	
/*TODO	//! Relaying laser scans (default: true)
	bool relay_scans;

	//! Publisher for the relayed scans (default topic: "/scan_relay")
	ros::Publisher relay_pub;
	
*/
	//! Min distance to keep point (default: 0)
	double min_distance;

	//! Time offset (default: 0)
	ros::Duration time_offset;

	//! min_angle to do shadow filtering (default: 0.1)
	double shadow_filter_min_angle;

	//! window size for the shadow filtering (default: 5)
	int shadow_filter_window;

	//! Scan filtering function
	void filter_scan(const sensor_msgs::LaserScan& scan);

	//! Scan callback function
	void scan_cb(const sensor_msgs::LaserScan& scan);

	//! Get the new scan in the point cloud
	void append_scan(const sensor_msgs::LaserScan& scan);
	
	//! Get laser angle from the tf
	double get_laser_angle(const ros::Time &time) const;

	//! Check if the robot moves
	bool check_no_motion(const ros::Time &time) const;

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

	// max number of point
	max_size = getParam<int>(n_, "max_size", 1000000);

	// channels
	point_cloud_channels = getParam<int>(n_, "channels",
			laser_geometry::channel_option::None);

	// 2d scans
	laser_angle_offset = getParam<double>(n, "laser_angle_offset", -0.035);
	publish2d = getParam<bool>(n_, "publish2d", true);
	if (publish2d) {
		std::string scan2d_topic;
		scan2d_topic = getParam<std::string>(n_, "scan2d_topic",
			"/scan2d");
		scan2d_pub = n.advertise<sensor_msgs::LaserScan>(scan2d_topic, 50);
	}

/* TODO	// relay
	relay_scans = getParam<bool>(n_, "relay_scans", true);
	if (relay_scans) {
		std::string relay_topic;
		relay_topic = getParam<std::string>(n_, "relay_topic",
			"/scan_relay");
		relay_pub = n.advertise<sensor_msgs::LaserScan>(relay_topic, 50);
	}
*/
	// moving
	start_time = ros::Time(0);
	publish_in_motion = getParam<bool>(n_, "publish_in_motion", true);

	// filtering
	double offset;
	offset = getParam<double>(n_, "time_offset", 0.0);
	time_offset = ros::Duration(offset);
	min_distance = getParam<double>(n_, "min_distance", 0.0);
	shadow_filter_min_angle = getParam<double>(n_, "shadow_filter_min_angle",
			0.1);
	shadow_filter_window = getParam<int>(n_, "shadow_filter_window", 5);
	// initialized so that the first test always fails
	previous_angle = NAN;

	if (!tf_listener.waitForTransform(laser_frame, world_frame, ros::Time(0),
			ros::Duration(10.)))
		ROS_WARN_STREAM("Timeout (10s) while waiting between "<<laser_frame<<
				" and "<<world_frame<<" at startup.");

	// point cloud publisher
	std::string point_cloud_topic;
	point_cloud_topic = getParam<std::string>(n_, "point_cloud_topic",
			"/nifti_point_cloud");
	point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			(point_cloud_topic, 50);
	
	// laser scan subscriber
	std::string laser_scan_topic;
	laser_scan_topic = getParam<std::string>(n_, "laser_scan_topic", "/scan");
	laser_scan_sub = n.subscribe(laser_scan_topic, 50,
			&NiftiLaserAssembler::scan_cb, this);
}


/*
 * Destructor
 */
NiftiLaserAssembler::~NiftiLaserAssembler()
{
	// Nothing to do?
}


// get angle (similar in laser_filters/scan_shadow_filter)
double get_angle(double r1, double r2, double gamma)
{
	// area->sin; dist->cos
	return M_PI/2-fabs(atan2(r1*sin(gamma), r2 - r1*cos(gamma))-M_PI/2);
}


/*
 * Scan filtering
 */
void NiftiLaserAssembler::filter_scan(const sensor_msgs::LaserScan& scan)
{
	tmp_scan = scan;
	tmp_scan.header.stamp = scan.header.stamp + time_offset;
	tmp_scan.range_min = std::max(scan.range_min, (float)min_distance);
	double invalid = -1.0;

	for (unsigned int i=1; i<scan.ranges.size(); i++) {
		for (unsigned int d=1; d<=(unsigned)shadow_filter_window; d++){
			if (i<d) continue;
			if (shadow_filter_min_angle > get_angle(scan.ranges[i-d],
					scan.ranges[i], d*scan.angle_increment)){
				if (scan.ranges[i-d]>scan.ranges[i]) // keep closer point
					tmp_scan.ranges[i-d] = invalid;
				else
					tmp_scan.ranges[i] = invalid;
			}
		}
	}
}

/*
 * laser scan callback
 */
void NiftiLaserAssembler::scan_cb(const sensor_msgs::LaserScan& scan)
{
	double angle = get_laser_angle(scan.header.stamp);
	//ROS_INFO_STREAM("Got scan");

	//filtering scan in all cases
	filter_scan(scan);

	if (relay_scans) {
		relay_pub.publish(tmp_scan);
	}
	
	if ((angle*previous_angle<=0.0) ||
			((angle==previous_angle)&&
					(fabs(angle+laser_angle_offset)<0.5*M_PI/180.))) {
		ROS_DEBUG_STREAM("Publishing 2d scan.");
		scan2d_pub.publish(tmp_scan);
	}

	if (fabs(angle)<=M_PI/2) {
		//ROS_INFO_STREAM("Got scan in range.");
		if (start_time.isZero())
			start_time = tmp_scan.header.stamp;
		append_scan(tmp_scan);

	}

	if ((fabs(previous_angle)<M_PI/2) && (fabs(angle)>=M_PI/2)) {
		//ROS_INFO_STREAM("End");
		if (publish_in_motion||check_no_motion(tmp_scan.header.stamp)){
			ROS_DEBUG_STREAM("Publishing point cloud (" << point_cloud.width << " points).");
			point_cloud_pub.publish(point_cloud);
		} else {
			ROS_DEBUG_STREAM("Dropping point cloud (in motion).");
		}
		point_cloud.data.clear();
		point_cloud.width = 0;
		start_time = ros::Time(0);
	}

	// if point cloud is full, we publish it
	// TODO decide if relevant
	if (point_cloud.width>=(unsigned)max_size) {
		ROS_WARN_STREAM("max_size exceeded, publishing.");
		point_cloud_pub.publish(point_cloud);
		point_cloud.data.clear();
		point_cloud.width = 0;
	}
	previous_angle = angle;
}


/*
 * Append a scan to the current point cloud
 */
void NiftiLaserAssembler::append_scan(const sensor_msgs::LaserScan& scan)
{
	// Project the LaserScan into a PointCloud2
	if (!tf_listener.waitForTransform(laser_frame, world_frame, scan.header.stamp
			+ ros::Duration ().fromSec (scan.scan_time)
			, ros::Duration(2.)))
		ROS_WARN_STREAM("Timeout (2s) while waiting between "<<laser_frame<<
				" and "<<world_frame<<" before transformation.");
	projector.transformLaserScanToPointCloud(world_frame, scan,
			tmp_point_cloud, tf_listener, point_cloud_channels);

	// Aggregate the new point cloud into the current stack
	if (point_cloud.width <= 0)
		point_cloud = tmp_point_cloud;
	else {
		// TODO check we don't go over max_size
		point_cloud.header = tmp_point_cloud.header;
		point_cloud.width += tmp_point_cloud.width;
		point_cloud.row_step += tmp_point_cloud.row_step;
		point_cloud.data.insert(point_cloud.data.end(),
				tmp_point_cloud.data.begin(), tmp_point_cloud.data.end());
	}
}


/*
 * Get the laser angle from the tf_listener
 */
double NiftiLaserAssembler::get_laser_angle(const ros::Time &time) const
{
	double angle;
	tf::StampedTransform tmp_tf;
	geometry_msgs::Quaternion rot;
	if (!tf_listener.waitForTransform(laser_frame, robot_frame, time,
		ros::Duration(1.)))
		ROS_WARN_STREAM("Timeout (1s) while waiting between "<<laser_frame<<
				" and "<<robot_frame<<" before getting laser angle.");
	try {
		tf_listener.lookupTransform(laser_frame, robot_frame, time, tmp_tf);
	} catch (tf::ExtrapolationException e) {
		tf_listener.lookupTransform(laser_frame, robot_frame, ros::Time(0),
				tmp_tf);
		ROS_WARN_STREAM(e.what() << "Trying with most recent.");
	}
		
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
	return ((norm(mean_speed.linear)*delta.toSec()<0.01)&&
			(norm(mean_speed.angular)*delta.toSec()<M_PI/180.));
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

