/*
	Created on: 	March 30th, 2014
	Author:		Kent Stark Olsen
	E-Mail:		kent.stark.olsen@gmail.com
*/

#include "point_cloud_assembler.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "point_cloud_assembler");

	PointCloudAssembler pca;
	pca.makeMeSpin();

	return 0;
}

PointCloudAssembler::PointCloudAssembler()
{
	//	Parameters (NodeHandle)
	this->nodeHandle = ros::NodeHandle("~");
	this->nodeHandle.param<int>("loop_rate", this->loopRate, 20);

	//	Parameters (Input)
	this->nodeHandle.param<std::string>("input_topic", this->input.topic, "/camera/depth/points");
	this->nodeHandle.param<int>("input_buffer_size", this->input.bufferSize, 10);

	//	Parameters (Output)
//	this->nodeHandle.param<std::string>("output_topic", this->output.topic, "processed_points");
//	this->nodeHandle.param<int>("output_buffer_size", this->output.bufferSize, 10);

	//	Setup Callback
	this->input.subscriber = this->nodeHandle.subscribe<sensor_msgs::PointCloud2>(this->input.topic, this->input.bufferSize, &PointCloudAssembler::pointCloudCallback, this);
}

PointCloudAssembler::~PointCloudAssembler(){}

void PointCloudAssembler::makeMeSpin(void)
{
	ros::Rate r(this->loopRate);

	while (ros::ok() && this->nodeHandle.ok())
	{
		//	 Update event queue
		ros::spinOnce();

		ROS_INFO("Spinning!!! PointCloud size: %d", (int)this->input.points.size());

		r.sleep();
	}
}

void PointCloudAssembler::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	pcl::fromROSMsg(*data, this->input.points);
}
