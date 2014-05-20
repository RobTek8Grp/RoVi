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

	//	Debug (aux. info)
	this->nodeHandle.param<bool>("debug", this->publishAuxPoints, true);
	this->nodeHandle.param<bool>("broadcast_transform", this->broadcastTf, true);
	this->auxPub = this->nodeHandle.advertise<sensor_msgs::PointCloud2>("debug/auxPoints", 10);

	//	Parameters (group4 msg)
	this->nodeHandle.param<std::string>("input_topic", this->groupMsg.topic, "/robot_rx60b/carmine_pose");
	this->groupMsg.lastPoseId = 0;

	//	Parameters (output msg)
	this->nodeHandle.param<std::string>("output_topic", this->outputMsg.topic, "assembled_point_cloud");
	this->outputMsg.isPointCloudAssembled = false;

	//	System parameters
	this->nodeHandle.param<double>("cutoff/x_min", this->systemParameters.cutOffFilterLimits.x.min, -.2);
	this->nodeHandle.param<double>("cutoff/x_max", this->systemParameters.cutOffFilterLimits.x.max, .2);
	this->nodeHandle.param<double>("cutoff/y_min", this->systemParameters.cutOffFilterLimits.y.min, -.2);
	this->nodeHandle.param<double>("cutoff/y_max", this->systemParameters.cutOffFilterLimits.y.max, .2);
	this->nodeHandle.param<double>("cutoff/z_min", this->systemParameters.cutOffFilterLimits.z.min, .5);
	this->nodeHandle.param<double>("cutoff/z_max", this->systemParameters.cutOffFilterLimits.z.max, 1.0);

	this->nodeHandle.param<double>("voxel/x", this->systemParameters.voxelLeafSize.x, 0.005);
	this->nodeHandle.param<double>("voxel/y", this->systemParameters.voxelLeafSize.y, 0.005);
	this->nodeHandle.param<double>("voxel/z", this->systemParameters.voxelLeafSize.z, 0.005);

	this->nodeHandle.param<double>("stitching/epsilon", this->systemParameters.stitching.epsilon, 1e-6);
	this->nodeHandle.param<double>("stitching/max_correspondance_distance", this->systemParameters.stitching.maxCorrespondenceDistance, 0.01);

	//	Transforms
	this->tf.setOrigin(tf::Vector3(0, 0, 0));
	this->tf.setRotation(tf::Quaternion(0, 0, 0, 1));

	//	Offset transform
	tf::Quaternion q;
	q.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);
//	this->tfOffset.setOrigin(tf::Vector3(-0.046, 0.052, 0.056));	//	Gotten from TF
	this->tfOffset.setOrigin(tf::Vector3(0.0, 0.052, 0.056));		//	Kent's calibration
	this->tfOffset.setRotation(q);

	//	Point Cloud Filters (Cut-Off)
	this->pcFilter.setCutOffLimits(	this->systemParameters.cutOffFilterLimits.x.min, this->systemParameters.cutOffFilterLimits.x.max,
									this->systemParameters.cutOffFilterLimits.y.min, this->systemParameters.cutOffFilterLimits.y.max,
									this->systemParameters.cutOffFilterLimits.z.min, this->systemParameters.cutOffFilterLimits.z.max);

	//	Voxel filter
	this->pcFilter.setVoxelLeafSizes(	this->systemParameters.voxelLeafSize.x,
										this->systemParameters.voxelLeafSize.y,
										this->systemParameters.voxelLeafSize.z);

	//	Point Cloud Stitching
	this->pcStitching = PointCloudStitching();

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

		if (this->broadcastTf)
			this->tfBroadcast.sendTransform(tf::StampedTransform(this->tf, ros::Time::now(), "base_link", "camera_depth_optical_frame"));

		this->publishMsg();

		r.sleep();
	}

	ROS_ERROR(" point_cloud_assembler: Failed to continue processing ...");
}

void PointCloudAssembler::msgCallback(const group4_msgs::PointCloudPose data)
{
	//ROS_INFO(" point_cloud_assembler: Message callback ...");

	this->groupMsg.data.push_back(group4_msgs::PointCloudPose(data));
}

void PointCloudAssembler::update(void)
{
	//ROS_INFO(" point_cloud_assembler: Updating ...");

	//	Process data from buffer when possible (one frame per update)
	if ((int)this->groupMsg.data.size() > 0)
	{
		group4_msgs::PointCloudPose f = this->groupMsg.data.at(0);

		//	Lock while manipulating buffer
		this->groupMsg.data.erase(this->groupMsg.data.begin());

		if ((f.pose_id.data - 1) == this->groupMsg.lastPoseId)
		{
			//	Process frame
			if (f.pose_id.data == 1)
				this->pcStitching.reset();

			//	Convert ros type to pcl type
			pcl::PointCloud<PointT> p;
			pcl::fromROSMsg(f.carmine_pointcloud, p);	//	TODO: Is chaniging to point cloud given by Frederik

			ROS_INFO(" point_cloud_assembler: Processing frame %d/%d ... ", f.pose_id.data, f.pose_id_max.data);
			this->processFrame(p, f.carmine_pose);		//	Process points

			if (f.pose_id.data == f.pose_id_max.data)
			{
				//ROS_INFO(" point_cloud_assembler: Resetting counter... ");
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
	//	Setup transform from pose
	double x = pose.position.x;
	double y = pose.position.y;
	double z = pose.position.z;
	double a = pose.orientation.x;
	double b = pose.orientation.y;
	double c = pose.orientation.z;
	double d = pose.orientation.w;

	tf::Transform t;
	t.setOrigin(tf::Vector3(x,y,z));
	t.setRotation(tf::Quaternion(a,b,c,d));

	this->tf = t * this->tfOffset;

	x = this->tf.getOrigin().getX();
	y = this->tf.getOrigin().getY();
	z = this->tf.getOrigin().getZ();

	a = this->tf.getRotation().getX();
	b = this->tf.getRotation().getY();
	c = this->tf.getRotation().getZ();
	d = this->tf.getRotation().getW();

	Eigen::Affine3d tfp = Eigen::Affine3d::Identity();
	Eigen::Quaterniond q(d,a,b,c);
	Eigen::Vector3d p(x,y,z);

	tfp.translate(p);
	tfp.rotate(q.matrix());

	//	Transform point cloud
	pcl::PointCloud<PointT> tfPoints, filteredPoints;
	pcl::transformPointCloud(points, tfPoints, tfp);

	//	Cut-off filter
	this->pcFilter.cutOffFilter<PointT>(tfPoints, filteredPoints);

	//	Aux. point cloud
	if (this->publishAuxPoints)
		pcl::toROSMsg(filteredPoints, this->auxPoints);

	//	Stitch new point cloud into stitching
	this->pcStitching.stitch(	filteredPoints,
								this->systemParameters.stitching.epsilon,
								this->systemParameters.stitching.maxCorrespondenceDistance);
	
	// 	Stitch two latest PC's
	Eigen::Matrix4f tf = this->calibration.stitch(	filteredPoints, this->systemParameters.stitching.epsilon, this->systemParameters.stitching.maxCorrespondenceDistance);
	this->calibration.setStitching(	filteredPoints );

	//	Horrible conversion - sorry Kent...
	tf::Matrix3x3 tf3d;
  	tf3d.setValue(	static_cast<double>(tf(0,0)), static_cast<double>(tf(0,1)), static_cast<double>(tf(0,2)), 
        		static_cast<double>(tf(1,0)), static_cast<double>(tf(1,1)), static_cast<double>(tf(1,2)), 
        		static_cast<double>(tf(2,0)), static_cast<double>(tf(2,1)), static_cast<double>(tf(2,2)));
	
	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);

	//	Publish error transform
	this->calibrationMsg.data.header.frame_id = this->groupMsg.data.pose_id.back()
	this->calibrationMsg.data.position.x = tf(0,3);
	this->calibrationMsg.data.position.y = tf(1,3);
	this->calibrationMsg.data.position.z = tf(2,3);

	this->calibrationMsg.data.orientation.x = tfqt.x();
	this->calibrationMsg.data.orientation.y = tfqt.y();
	this->calibrationMsg.data.orientation.z = tfqt.z();
	this->calibrationMsg.data.orientation.w = tfqt.w();

	//	Voxel filter after stitching
	pcl::PointCloud<PointT> voxelFiltered, stitched(*this->pcStitching.getStitching());
	this->pcFilter.voxelFilter<PointT>(stitched, voxelFiltered);
	this->pcStitching.setStitching(voxelFiltered);
}

void PointCloudAssembler::publishMsg(void)
{
	if (this->outputMsg.isPointCloudAssembled)
	{
		ROS_INFO(" point_cloud_assembler: Publishing assembled point cloud (Size: %d) ...", (int)(this->pcStitching.getStitching().get()->size()));

		pcl::toROSMsg(*this->pcStitching.getStitching(), this->outputMsg.data);

		this->outputMsg.isPointCloudAssembled = false;
	}

	this->outputMsg.data.header.stamp = ros::Time::now();
	this->outputMsg.data.header.frame_id = "base_link";
	this->outputMsg.pub.publish(this->outputMsg.data);

	this->calibrationMsg.data.header.stamp = ros::Time::now();
	this->calibrationMsg.pub.publish(this->calibrationMsg.data);

	if (this->publishAuxPoints)
	{
		this->auxPoints.header.stamp = ros::Time::now();
		this->auxPoints.header.frame_id = "base_link";
		this->auxPub.publish(this->auxPoints);
	}
}
