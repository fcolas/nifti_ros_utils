
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
#include <geometry_msgs/Twist.h>
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

	//! Point cloud holding the projection of one laser scan
	sensor_msgs::PointCloud2 tmp_point_cloud;

	//! Previous absolute value of the laser angle (to detect when to publish)
	double previous_angle;

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
	n_("~")
{
	// frame names
	n_.param<std::string>("laser_frame", laser_frame, "/laser");
	n_.param<std::string>("robot_frame", robot_frame, "/base_link");
	n_.param<std::string>("world_frame", world_frame, "/odom");

	// point cloud publisher
	std::string point_cloud_topic;
	n_.param<std::string>("point_cloud_topic", point_cloud_topic,
			"/nifti_point_cloud");
	point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			(point_cloud_topic, 50);
	
	// laser scan subscriber
	std::string laser_scan_topic;
	n_.param<std::string>("laser_scan_topic", laser_scan_topic, "/scan");
	laser_scan_sub = n.subscribe(laser_scan_topic, 50,
			&NiftiLaserAssembler::scan_cb, this);

	// max number of point
	n_.param<int>("max_size", max_size, 1000000);

	// channels
	n_.param<int>("channels", point_cloud_channels,
			laser_geometry::channel_option::None);

	// 2d scans
	n_.param<bool>("publish2d", publish2d, true);
	if (publish2d) {
		std::string scan2d_topic;
		n.param<std::string>("scan2d_topic", scan2d_topic,
			"/scan2d");
		scan2d_pub = n.advertise<sensor_msgs::LaserScan>(scan2d_topic, 50);
	}

	// moving
	start_time = ros::Time(0);
	n_.param<bool>("publish_in_motion", publish_in_motion, true);
	ROS_WARN_STREAM(publish_in_motion);

	// initialized so that the first test always fails
	previous_angle = NAN;
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
	double angle = get_laser_angle(scan.header.stamp);

	if ((angle*previous_angle<=0.0) ||
			((angle==previous_angle)&&(fabs(angle)<0.5*M_PI/180.))) {
		ROS_INFO_STREAM("Publishing 2d scan.");
		scan2d_pub.publish(scan);
	}

	if (fabs(angle)<=M_PI/2) {
		//ROS_INFO_STREAM("Got scan in range.");
		if (start_time.isZero())
			start_time = scan.header.stamp;
		append_scan(scan);

	}

	if ((fabs(previous_angle)<M_PI/2) && (fabs(angle)>=M_PI/2)) {
		if (publish_in_motion||check_no_motion(scan.header.stamp)){
			ROS_INFO_STREAM("Publishing scan (" << point_cloud.width << " points).");
			point_cloud_pub.publish(point_cloud);
		} else {
			ROS_INFO_STREAM("Dropping scan.");
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
	tf_listener.waitForTransform(laser_frame, world_frame, scan.header.stamp
			+ ros::Duration ().fromSec (scan.scan_time)
			, ros::Duration(1.));
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
	tf_listener.waitForTransform(laser_frame, robot_frame, time,
		ros::Duration(1.));
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
	tf_listener.waitForTransform(robot_frame, world_frame, time,
		ros::Duration(1.));
	tf_listener.lookupTwist(robot_frame, world_frame, start_time+delta*0.5,
			delta, mean_speed);
	//ROS_WARN_STREAM(norm(mean_speed.linear) << " " << norm(mean_speed.angular)
	//		<< " " <<delta.toSec()); 
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

