/*
	Created on:	March 30th, 2014
	Author:		Kent Stark Olsen
	E-Mail: 	kent.stark.olsen@gmail.com
*/

#ifndef POINTCLOUDASSEMBLER_H_
#define POINTCLOUDASSEMBLER_H_

#include <string.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//#include "point_cloud_cutoff.h"

typedef pcl::PointXYZ PointT;

class PointCloudAssembler
{
	//	Node Handle
	ros::NodeHandle nodeHandle;
	int loopRate;

	//	Input
	struct
	{
		pcl::PointCloud<PointT> points;
		ros::Subscriber subscriber;
		std::string topic;
		int bufferSize;
	} input;

	//	Output
/*	struct
	{
		pcl::PointCloud<PointT> points;
		ros::Publisher publisher;
		std::string topic;
		int bufferSize;
	} output;
*/
	//	Callbacks
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& data);

public:
	PointCloudAssembler();
	virtual ~PointCloudAssembler();

	void makeMeSpin(void);
};

#endif	/* POINTCLOUDASSEMBLER_H_ */
