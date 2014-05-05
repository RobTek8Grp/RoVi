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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class PointNormal : public pcl::PointRepresentation <pcl::PointNormal>
{
	using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

public:
	PointNormal ()
  	{
		//	Define the number of dimensions
		nr_dimensions_ = 4;
  	}

	//	Override the copyToFloatArray method to define feature vector
	virtual void copyToFloatArray (const pcl::PointNormal& p, float *out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

template <class _type> class PointCloudStitching
{
private:

public:
	PointCloudStitching();
	virtual ~PointCloudStitching();
};

#endif /* POINTCLOUDSTITCHING_HPP_ */
