/*
 * PointCloudStitching.cpp
 *
 *  Created on: May 5, 2014
 *      Author: kent
 */

#include "PointCloudStitching.hpp"

PointCloudStitching::PointCloudStitching()
{
	this->reset();
}

PointCloudStitching::~PointCloudStitching()
{

}

void PointCloudStitching::reset(void)
{
	this->stitching = pcl::PointCloud<PointT>();
}

pcl::PointCloud<PointT>::ConstPtr PointCloudStitching::getStitching(void)
{
	return this->stitching.makeShared();
}

int PointCloudStitching::stitch(pcl::PointCloud<PointT>& points)
{
	if (this->stitching.size() == 0)
	{
		pcl::copyPointCloud(points, this->stitching);
		return 0;
	}

	//	Compute surface normals and curvature
	pcl::PointCloud<PointT> tempTarget;
	pcl::copyPointCloud(this->stitching, tempTarget);

	pcl::PointCloud<pcl::PointNormal>::Ptr pointsWithNormalSource (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pointsWithNormalTarget (new pcl::PointCloud<pcl::PointNormal>);

	pcl::NormalEstimation<PointT, pcl::PointNormal> normalEstimate;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	normalEstimate.setSearchMethod(tree);
	normalEstimate.setKSearch(30);

	normalEstimate.setInputCloud(points.makeShared());
	normalEstimate.compute(*pointsWithNormalSource);
	pcl::copyPointCloud(points, *pointsWithNormalSource);

	normalEstimate.setInputCloud (tempTarget.makeShared());
	normalEstimate.compute(*pointsWithNormalTarget);
	pcl::copyPointCloud (tempTarget, *pointsWithNormalTarget);

	//	Instantiate custom point representation
	MyPointNormal pointNormal;
	//	... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	pointNormal.setRescaleValues(alpha);

	//	Align
	pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> registration;
	registration.setTransformationEpsilon(1e-5);
	//	Set the maximum distance between two correspondences
	registration.setMaxCorrespondenceDistance(0.02);
	//	Set the point representation
	registration.setPointRepresentation (boost::make_shared<const MyPointNormal> (pointNormal));

	registration.setInputSource(pointsWithNormalSource);
	registration.setInputTarget(pointsWithNormalTarget);
	registration.setMaximumIterations(30);

	PCL_INFO("Sizes:  %d  --  %d", (int)pointsWithNormalSource.get()->size(), (int)pointsWithNormalTarget.get()->size());

	Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointNormal>::Ptr regResult = pointsWithNormalSource;
	registration.align(*regResult);
	tf = registration.getFinalTransformation().inverse();

	pcl::transformPointCloud(*pointsWithNormalTarget, *regResult, tf);
	*regResult += *pointsWithNormalSource;

	pcl::copyPointCloud(*regResult, this->stitching);

	return 0;
}
