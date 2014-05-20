/*
 * SurfaceReconstruction.hpp
 *
 *  Created on: May 6, 2014
 *      Author: kent
 */

#ifndef SURFACERECONSTRUCTION_HPP_
#define SURFACERECONSTRUCTION_HPP_

#include <vector>

#include <strings.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

class SurfaceReconstruction
{
private:
	ros::NodeHandle nodeHandle;
	int spinRate;

	struct
	{
		std::string topic;
		sensor_msgs::PointCloud2::Ptr data;
		ros::Subscriber sub;
	} pointCloud;

	//	Reconstruction
	bool noCloudReceived;
	bool reconstructionDone;
	pcl::PointCloud<pcl::PointXYZ> points;
	pcl::PointCloud<pcl::PointXYZINormal> pointNormals;
//	Reconstruction reconstruction;

	//	Visualizer
	pcl::visualization::PCLVisualizer::Ptr viewer;

	void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& data);

	//	Aux. function
	int prepareData (void);

public:
	SurfaceReconstruction();
	virtual ~SurfaceReconstruction();

	void makeMeSpin(void);
};

#endif /* SURFACERECONSTRUCTION_HPP_ */
