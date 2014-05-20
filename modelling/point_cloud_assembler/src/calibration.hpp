/*
 * PointCloudStitching.hpp
 *
 *  Created on: May 5, 2014
 *      Author: kent
 */

#ifndef CALIBRATION_HPP_
#define CALIBRATION_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "MyPointNormal.hpp"

typedef pcl::PointXYZ PointT;

class Calibration
{
private:
	pcl::PointCloud<PointT> stitching;

public:
	Calibration();
	virtual ~Calibration();

	void reset(void);
	pcl::PointCloud<PointT>::ConstPtr getStitching(void);
	void setStitching(pcl::PointCloud<PointT>& points);
	Eigen::Matrix4f  stitch(pcl::PointCloud<PointT>& points, double epsilon, double maxCorrespondanceDistance);
};

#endif /* POINTCLOUDSTITCHING_HPP_ */
