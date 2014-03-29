/*
 * PointCloudToSurface.cpp
 *
 *  Created on: Mar 17, 2014
 *      Author: kent
 */

#include "PointCloudToSurface.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudToSurface");

	PointCloudToSurface pctsNode;
	pctsNode.makeItSpin();

	return 0;
}

PointCloudToSurface::PointCloudToSurface() : 	viewer(new pcl::visualization::PCLVisualizer("3D Visualizer")),
													cloud(new pcl::PointCloud<PointT>())
{
	//	ROS node handler stuff
	this->nodeHandler = ros::NodeHandle("~");
	this->nodeHandler.param<int>("loop_rate", this->loopRate, 20);

	//	Setup system input
	this->nodeHandler.param<std::string>("cloud_topic", this->input.cloudTopic, "/camera/depth_registered/points");
	this->input.subscriber = this->nodeHandler.subscribe<sensor_msgs::PointCloud2>(this->input.cloudTopic, 10, &PointCloudToSurface::pointCloudCallback, this);

	//	Visualizer
	this->viewer->registerKeyboardCallback(&PointCloudToSurface::keyboardCallback, *this, (void*)&viewer);
	this->viewer->addCoordinateSystem(1.0,0);
	this->viewer->initCameraParameters();
}

PointCloudToSurface::~PointCloudToSurface() {
	// TODO Auto-generated destructor stub
}

void PointCloudToSurface::makeItSpin()
{
	ros::Rate r(this->loopRate);

//	//	Add shpere
//	PointT p = PointT(1.0,1.0,1.0);
//	this->viewer->addSphere(p, 1.0, "sphere", 0);

	while ((!this->viewer->wasStopped()) && ros::ok() && this->nodeHandler.ok())
	{
		//	Update event queue
		ros::spinOnce();

		//	Do stuff
		if (this->cloud->size())
		{
//			printf("\n Reading: \n");
//			printf(" ================================= \n");
//			printf(" Height: %d         Width: %d\n", this->cloud->height, this->cloud->width);
//			printf(" IsDense: %d  IsOrganized: %d\n", this->cloud->is_dense, this->cloud->isOrganized());
//			printf(" Number of points: %d\n", this->cloud->size());


			this->viewer->removePointCloud("cloud", 0);
			this->viewer->addPointCloud<PointT>(this->cloud, "cloud", 0);
			this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		}

		//	Update viewport
		this->viewer->spinOnce(1, true);

		//	Delay looping according to rate
		r.sleep();
	}

	printf("\n Closing visualizer and ROS node .... \n\n");
}

void PointCloudToSurface::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	pcl::fromROSMsg(*data, *this->cloud);
}

void PointCloudToSurface::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* userdata)
{
	ROS_INFO(" Keyboard callback!!! ");
}
