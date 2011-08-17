
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
//#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
//#include <std_msgs/Time.h> // needed?


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

	//! NodeHandle
	ros::NodeHandle n;

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

	//! Point cloud holding the projection of one laser scan
	sensor_msgs::PointCloud2 tmp_point_cloud;

	//! Previous absolute value of the laser angle (to detect when to publish)
	double previous_abs_angle;

	//! Projector object from LaserScan to PointCloud2
	laser_geometry::LaserProjection projector;

	/** \brief Set of channel to add to the point cloud (default: none)
	 *
	 * This follows the laser_geometry::channel_option::ChannelOption enum.
	 * By default, we don't add any channel. Parameter ~channels can be set to
	 * change that.
	 */
	int point_cloud_channels;

	//! Scan callback function
	void scan_cb(const sensor_msgs::LaserScan& scan);

	//! Get the new scan in the point cloud
	void append_scan(const sensor_msgs::LaserScan& scan);
	
	//! Get laser angle from the tf
	double get_laser_angle(const ros::Time &time) const;

};



/*
 * Constructor
 */
NiftiLaserAssembler::NiftiLaserAssembler()
{
	// frame names
	n.param<std::string>("laser_frame", laser_frame, "/laser");
	n.param<std::string>("robot_frame", robot_frame, "/base_link");
	n.param<std::string>("world_frame", world_frame, "/odom");

	// point cloud publisher
	std::string point_cloud_topic;
	n.param<std::string>("point_cloud_topic", point_cloud_topic,
			"/nifti_point_cloud");
	point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			(point_cloud_topic, 50);
	
	// laser scan subscriber
	std::string laser_scan_topic;
	n.param<std::string>("laser_scan_topic", laser_scan_topic, "/scan");
	laser_scan_sub = n.subscribe(laser_scan_topic, 10,
			&NiftiLaserAssembler::scan_cb, this);

	// max number of point
	n.param<int>("max_size", max_size, 1000000);

	// channels
	n.param<int>("channels", point_cloud_channels,
			laser_geometry::channel_option::None);

	// initialized so that the first test always fails
	previous_abs_angle = NAN;
}


/*
 * Destructor
 */
NiftiLaserAssembler::~NiftiLaserAssembler()
{
	// Nothing to do?
}


/*
 * laser scan callback
 */
void NiftiLaserAssembler::scan_cb(const sensor_msgs::LaserScan& scan)
{
	double abs_angle = fabs(get_laser_angle(scan.header.stamp));

	if (abs_angle<=M_PI/2) {
		//ROS_INFO_STREAM("Got scan in range.");
		append_scan(scan);
	}

	if ((previous_abs_angle<M_PI/2) && (abs_angle>=M_PI/2)) {
		ROS_INFO_STREAM("Publishing scan.");
		point_cloud_pub.publish(point_cloud);
		point_cloud.data.clear();
		point_cloud.width = 0;
	}

	// if point cloud is full, we publish it
	// TODO decide if relevant
	if (point_cloud.width>= max_size) {
		ROS_WARN_STREAM("max_size exceeded, publishing.");
		point_cloud_pub.publish(point_cloud);
		point_cloud.data.clear();
		point_cloud.width = 0;
	}
	previous_abs_angle = abs_angle;
}


/*
 * Append a scan to the current point cloud
 */
void NiftiLaserAssembler::append_scan(const sensor_msgs::LaserScan& scan)
{
	// Project the LaserScan into a PointCloud2
	tf_listener.waitForTransform(laser_frame, world_frame, scan.header.stamp
			+ ros::Duration ().fromSec (scan.scan_time)
			, ros::Duration(100.));
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
	//geometry_msgs::TransformStamped tmp_tf;
	tf_listener.waitForTransform(laser_frame, robot_frame, time,
		ros::Duration(10.));
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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "nifti_laser_assembler");

	NiftiLaserAssembler nla;

	ros::spin();

	return 0;
}

