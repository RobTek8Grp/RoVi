/*
 * SurfaceReconstruction.cpp
 *
 *  Created on: May 6, 2014
 *      Author: kent
 */

#include "SurfaceReconstruction.hpp"
#include "Reconstruction.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "surface_reconstruction");

	SurfaceReconstruction sr;
	sr.makeMeSpin();

	return -1;
}

SurfaceReconstruction::SurfaceReconstruction() : nodeHandle("~")//, viewer(new pcl::visualization::PCLVisualizer("Point Cloud/Surface viewer"))
{
	//	Node specifics
	this->nodeHandle.param<int>("spin_rate", this->spinRate, 1);

	this->noCloudReceived = true;
	this->reconstructionDone = false;

	//	Point cloud (input)
	this->nodeHandle.param<std::string>("point_cloud_topic", this->pointCloud.topic, "/point_cloud_assembler/assembled_point_cloud");

	//	Subscriber
	this->pointCloud.sub = this->nodeHandle.subscribe<sensor_msgs::PointCloud2>(this->pointCloud.topic, 10, &SurfaceReconstruction::msgCallback, this);
}

SurfaceReconstruction::~SurfaceReconstruction()
{
	// TODO Auto-generated destructor stub
}

void SurfaceReconstruction::msgCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	if (data.get()->data.size() && this->noCloudReceived)
	{
		pcl::fromROSMsg(*data, this->points);
		ROS_INFO(" surface_reconstruction: Point cloud received for reconstruction ...");
		this->noCloudReceived = false;
	}
}

int SurfaceReconstruction::prepareData(void)
{
//	//	Convert from ROS to PCL
//	pcl::PointCloud<pcl::PointXYZ>::Ptr p;
//	pcl::fromROSMsg(*this->pointCloud.data, *p);
//
//	//	Normal estimation
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointXYZINormal> normals;
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>());
//
//	normals.setInputCloud(this->points);
//	normals.setSearchMethod(kdTree);
//	normals.setRadiusSearch(0.05);
//
//	//	Compute
//	normals.compute(this->pointNormals);

	return 0;
}

void SurfaceReconstruction::makeMeSpin(void)
{
	//	Setup spin rate
	ros::Rate r(this->spinRate);

	//	Setup visualizer
//	this->viewer->addCoordinateSystem(1.0,0);
//	this->viewer->initCameraParameters();

	//	Spin
	while (ros::ok() && this->nodeHandle.ok())
	{
		//	 Update event queue
		ros::spinOnce();

		if (!this->noCloudReceived && !this->reconstructionDone)
		{
			ROS_INFO(" surface_reconstruction: Saving PCD-file ...");

			this->reconstructionDone = true;

			pcl::io::savePCDFileASCII("/home/kent/test.pcd", this->points);
		}

		r.sleep();
	}

	ROS_ERROR(" surface_reconstruction: Failed to continue processing ...");

}
