/*
 * MyPointNormal.hpp
 *
 *  Created on: May 5, 2014
 *      Author: kent
 */

#ifndef MYPOINTNORMAL_HPP_
#define MYPOINTNORMAL_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/features/normal_3d.h>

class MyPointNormal : public pcl::PointRepresentation <pcl::PointNormal>
{
	using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

public:
	MyPointNormal()
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

#endif /* MYPOINTNORMAL_HPP_ */
