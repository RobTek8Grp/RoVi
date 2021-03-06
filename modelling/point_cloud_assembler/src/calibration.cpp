/*
 * PointCloudStitching.cpp
 *
 *  Created on: May 5, 2014
 *      Author: kent
 */

#include "calibration.hpp"

Calibration::Calibration()
{
	this->reset();
}

Calibration::~Calibration()
{

}

void Calibration::reset(void)
{
	this->stitching = pcl::PointCloud<PointT>();
}

pcl::PointCloud<PointT>::ConstPtr Calibration::getStitching(void)
{
	return this->stitching.makeShared();
}

void Calibration::setStitching(pcl::PointCloud<PointT>& points)
{
	this->stitching = pcl::PointCloud<PointT>(points);
}


Eigen::Matrix4f Calibration::stitch(pcl::PointCloud<PointT>& points, double epsilon, double maxCorrespondanceDistance)
{
	if (this->stitching.size() == 0)
	{
		pcl::copyPointCloud(points, this->stitching);
		return Eigen::Matrix4f::Identity(); // Hardcore hack !!
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
	registration.setTransformationEpsilon(epsilon);
	//	Set the maximum distance between two correspondences
	registration.setMaxCorrespondenceDistance(maxCorrespondanceDistance);
	//	Set the point representation
	registration.setPointRepresentation (boost::make_shared<const MyPointNormal> (pointNormal));

	registration.setInputSource(pointsWithNormalSource);
	registration.setInputTarget(pointsWithNormalTarget);
	registration.setMaximumIterations(30);

	PCL_ERROR("Source size: %d  --  Target size: %d\n", (int)pointsWithNormalSource.get()->size(), (int)pointsWithNormalTarget.get()->size());

	Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointNormal>::Ptr regResult = pointsWithNormalSource;

	PCL_ERROR("Stitching ... ");
	registration.align(*regResult);
	PCL_ERROR("Done!\n");

	tf = registration.getFinalTransformation().inverse();

//	PCL_ERROR("\nTransform:\n");
//	PCL_ERROR("| %f %f %f %f |\n", tf(0,0), tf(0,1), tf (0,2), tf(0,3));
//	PCL_ERROR("| %f %f %f %f |\n", tf(1,0), tf(1,1), tf (1,2), tf(1,3));
//	PCL_ERROR("| %f %f %f %f |\n", tf(2,0), tf(2,1), tf (2,2), tf(2,3));
//	PCL_ERROR("| %f %f %f %f |\n\n", tf(3,0), tf(3,1), tf (3,2), tf(3,3));

	pcl::transformPointCloud(*pointsWithNormalSource, *regResult, tf);
	*regResult += *pointsWithNormalTarget;

	pcl::copyPointCloud(*regResult, this->stitching);

	return tf;
}

