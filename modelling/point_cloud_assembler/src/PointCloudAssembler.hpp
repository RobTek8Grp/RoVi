/*
 * PointCloudAssembler.hpp
 *
 *  Created on: May 2, 2014
 *      Author: kent
 */

#ifndef POINTCLOUDASSEMBLER_HPP_
#define POINTCLOUDASSEMBLER_HPP_

#include <string.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <group4_msgs/PointCloudPose.h>

#include "PointCloudFilter.hpp"
#include "PointCloudStitching.hpp"
#include "calibration.hpp"

typedef pcl::PointXYZ PointT;

class PointCloudAssembler
{
private:
	//	Node specifics
	ros::NodeHandle nodeHandle;

	int spinRate;
	bool publishAuxPoints, broadcastTf;
	ros::Publisher auxPub;
	sensor_msgs::PointCloud2 auxPoints;

	//	Aux. variables
	tf::TransformBroadcaster tfBroadcast;
	tf::Transform tf, tfOffset;
	PointCloudFilter pcFilter;
	PointCloudStitching pcStitching;
	Calibration calibration;

	//	Structs for system- parameters, input and output
	struct
	{
		struct
		{
			struct
			{
				double min;
				double max;
			} x;

			struct
			{
				double min;
				double max;
			} y;

			struct
			{
				double min;
				double max;
			} z;
		} cutOffFilterLimits;

		struct
		{
			double x;
			double y;
			double z;
		} voxelLeafSize;

		struct
		{
			double epsilon;
			double maxCorrespondenceDistance;
		} stitching;;
	} systemParameters;

	struct
	{
		ros::Subscriber sub;
		std::string topic;
		std::vector<group4_msgs::PointCloudPose> data;

		int lastPoseId;
	} groupMsg;

	struct
	{
		ros::Publisher pub;
		std::string topic;
		sensor_msgs::PointCloud2 data;

		bool isPointCloudAssembled;
	} outputMsg;

	struct
	{
		ros::Publisher pub;
		std::string topic;
		geometry_msgs::PoseStamped data;
	} calibrationMsg;

	//	Callback
	void msgCallback(const group4_msgs::PointCloudPose data);

	//	Functions
	void update(void);
	void processFrame(pcl::PointCloud<PointT>& points, geometry_msgs::Pose& pose);
	void publishMsg(void);

public:
	PointCloudAssembler();
	virtual ~PointCloudAssembler();

	void makeMeSpin (void);
};

#endif /* POINTCLOUDASSEMBLER_HPP_ */
