/*
 * PointCloudToSurface.h
 *
 *  Created on: Mar 17, 2014
 *      Author: kent
 */

#ifndef POINTCLOUDTOSURFACE_H_
#define POINTCLOUDTOSURFACE_H_

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_cloud_handlers.h>

typedef pcl::PointXYZ PointT;

class PointCloudToSurface
{
	//	Node handler
	ros::NodeHandle nodeHandler;
	int loopRate;

	//	Boost
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr< pcl::PointCloud<PointT> > cloud;

	//	System input
	struct
	{
		pcl::PointCloud<PointT> pointCloud;

		ros::Subscriber subscriber;
		std::string cloudTopic;
	} input;

	//	System output
	struct
	{

	} output;

	//	Callback
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& data);
	void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

public:
	PointCloudToSurface();
	virtual ~PointCloudToSurface();

	void makeItSpin (void);
};

#endif /* POINTCLOUDTOSURFACE_H_ */
