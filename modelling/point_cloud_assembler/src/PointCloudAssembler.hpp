/*
 * PointCloudAssembler.hpp
 *
 *  Created on: May 2, 2014
 *      Author: kent
 */

#ifndef POINTCLOUDASSEMBLER_HPP_
#define POINTCLOUDASSEMBLER_HPP_

#include <mutex>

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

typedef pcl::PointXYZ PointT;

class PointCloudAssembler
{
private:
	ros::NodeHandle nodeHandle;
	int spinRate;

	std::mutex tLock;
	tf::TransformBroadcaster tfBroadcast;
	tf::Transform tf, tfOffset;
	PointCloudFilter pcFilter;
	PointCloudStitching pcStitching;

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
		} voxelLeafSizesBS;

		struct
		{
			double x;
			double y;
			double z;
		} voxelLeafSizesAS;
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

	void msgCallback(const group4_msgs::PointCloudPose data);

	void update(void);
	void processFrame(pcl::PointCloud<PointT>& points, geometry_msgs::Pose& pose);
	void publishMsg(void);

public:
	PointCloudAssembler();
	virtual ~PointCloudAssembler();

	void makeMeSpin (void);
};

#endif /* POINTCLOUDASSEMBLER_HPP_ */
