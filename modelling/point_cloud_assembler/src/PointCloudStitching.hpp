/*
 * PointCloudStitching.hpp
 *
 *  Created on: May 5, 2014
 *      Author: kent
 */

#ifndef POINTCLOUDSTITCHING_HPP_
#define POINTCLOUDSTITCHING_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "MyPointNormal.hpp"

typedef pcl::PointXYZ PointT;

class PointCloudStitching
{
private:
	pcl::PointCloud<PointT> stitching;

	void reset(void);

public:
	PointCloudStitching();
	virtual ~PointCloudStitching();

	pcl::PointCloud<PointT>::ConstPtr getStitching(void);
	int stitch(pcl::PointCloud<PointT>& points);
};

#endif /* POINTCLOUDSTITCHING_HPP_ */
