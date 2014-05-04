/*
 * PointCloudAssembler.cpp
 *
 *  Created on: May 2, 2014
 *      Author: kent
 */

#include "PointCloudAssembler.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "point_cloud_assembler");

	PointCloudAssembler pca;
	pca.makeMeSpin();

	return -1;
}

PointCloudAssembler::PointCloudAssembler() : nodeHandle("~")
{
	this->nodeHandle.param<int>("spin_rate", this->spinRate, 2);

	this->tf.setOrigin(tf::Vector3(0, 0, 0));
	this->tf.setRotation(tf::Quaternion(0, 0, 0, 1));

	//	Parameters (group4 msg)
	this->nodeHandle.param<std::string>("input_topic", this->groupMsg.topic, "/robot_rx60b/carmine_pose");
	this->groupMsg.lastPoseId = 0;

	//	Parameters (output msg)
	this->nodeHandle.param<std::string>("output_topic", this->outputMsg.topic, "assembled_point_cloud");
	this->outputMsg.isPointCloudAssembled = false;

	//	Setup callback
	this->groupMsg.sub = this->nodeHandle.subscribe<group4_msgs::PointCloudPose>(this->groupMsg.topic, 10, &PointCloudAssembler::msgCallback, this);

	//	Setup publisher
	this->outputMsg.pub = this->nodeHandle.advertise<sensor_msgs::PointCloud2>(this->outputMsg.topic, 10);
}

PointCloudAssembler::~PointCloudAssembler() {
	// TODO Auto-generated destructor stub
}

void PointCloudAssembler::makeMeSpin(void)
{
	ros::Rate r(this->spinRate);

	while (ros::ok() && this->nodeHandle.ok())
	{
		//	 Update event queue
		ros::spinOnce();

		this->update();

		//	Braodcast temporary transform
		this->tfBroadcast.sendTransform(tf::StampedTransform(this->tf, ros::Time::now(), "camera_depth_optical_frame", "base_link"));

		this->publishMsg();

		r.sleep();
	}

	ROS_ERROR(" point_cloud_assembler: Failed to continue processing ...");
}

void PointCloudAssembler::msgCallback(const group4_msgs::PointCloudPose data)
{
	ROS_INFO(" point_cloud_assembler: Message callback ...");

	this->groupMsg.data.push_back(group4_msgs::PointCloudPose(data));
}

void PointCloudAssembler::update(void)
{
	ROS_INFO(" point_cloud_assembler: Updating ...");

	//	Process data from buffer when possible (one frame per update)
	if ((int)this->groupMsg.data.size() > 0)
	{
		group4_msgs::PointCloudPose f = this->groupMsg.data.at(0);

		//	Lock while manipulating buffer
		this->tLock.lock();
		this->groupMsg.data.erase(this->groupMsg.data.begin());
		this->tLock.unlock();

		if ((f.pose_id.data - 1) == this->groupMsg.lastPoseId)
		{
			//	Process frame
			ROS_INFO(" point_cloud_assembler: Processing frame %d/%d ... ", f.pose_id.data, f.pose_id_max.data);

			//	Convert ros type to pcl type
			pcl::PointCloud<PointT> p;
			pcl::fromROSMsg(f.carmine_pointcloud, p);	//	TODO: Is chaniging to point cloud given by Frederik

			this->processFrame(p, f.carmine_pose);		//	Process points


			if (f.pose_id.data == f.pose_id_max.data)
			{
				ROS_INFO(" point_cloud_assembler: Resetting counter... ");
				this->groupMsg.lastPoseId = 0;

				this->outputMsg.isPointCloudAssembled = true;
			}
			else
			{
				this->groupMsg.lastPoseId++;
			}
		}
	}
}

void PointCloudAssembler::processFrame(pcl::PointCloud<PointT>& points, geometry_msgs::Pose& pose)
{
	ROS_INFO(" point_cloud_assembler: Processing frame ...");
//	ROS_INFO(" point_cloud_assembler: Height: %d  -  Width: %d", (int)this->groupMsg.data->carmine_pointcloud.height, (int)this->groupMsg.data->carmine_pointcloud.width);

	//	Setup transform from pose
	double a = pose.orientation.x;
	double b = pose.orientation.y;
	double c = pose.orientation.z;
	double d = pose.orientation.w;

//	this->tf.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
//	this->tf.setRotation(tf::Quaternion(a, b, c, d));

	Eigen::Affine3d t = Eigen::Affine3d();
	t.setIdentity();
	t(0,3) = pose.position.x;
	t(1,3) = pose.position.y;
	t(2,3) = pose.position.z;

	t(0,0) = a * a + b * b - c * c - d * d;
	t(0,1) = 2 * b * c - 2 * a * d;
	t(0,2) = 2 * b * d + 2 * a * c;
	t(1,0) = 2 * b * c + 2 * a * d;
	t(1,1) = a * a - b * b + c * c - d * d;
	t(1,2) = 2 * c * d - 2 * a * b;
	t(2,0) = 2 * b * d - 2 * a * c;
	t(2,1) = 2 * c * d - 2 * a * b;
	t(2,2) = a * a - b * b - c * c + d * d;

	//	Extract point cloud from msg
	pcl::PointCloud<PointT> tfPoints;
	pcl::transformPointCloud(points, tfPoints, t);

	ROS_INFO(" point_cloud_assembler: Point cloud size: %d", (int)points.size());

	//	Setup output point cloud
	pcl::toROSMsg(tfPoints, this->outputMsg.data);

	this->outputMsg.isPointCloudAssembled = true;
}

void PointCloudAssembler::publishMsg(void)
{
	if (this->outputMsg.isPointCloudAssembled)
	{
		ROS_INFO(" point_cloud_assembler: Publishing cloud ...");
		this->outputMsg.data.header.stamp = ros::Time::now();
		this->outputMsg.data.header.frame_id = "base_link";
		this->outputMsg.pub.publish(this->outputMsg.data);
		this->outputMsg.isPointCloudAssembled = false;
	}
}
