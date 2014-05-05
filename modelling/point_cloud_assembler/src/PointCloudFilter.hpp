/*
 * PointCloudFilter.hpp
 *
 *  Created on: May 4, 2014
 *      Author: kent
 */

#ifndef POINTCLOUDFILTER_HPP_
#define POINTCLOUDFILTER_HPP_

#include <pcl/filters/passthrough.h>

class PointCloudFilter
{
private:
	struct
	{
		struct
		{
			double min;
			double max;
		} x;
		struct
		{
			double min;
			double max;
		} y;
		struct
		{
			double min;
			double max;
		} z;
	} limits;

public:
	PointCloudFilter()
	{
		this->limits.x.min = -1.0;
		this->limits.x.max = 1.0;
		this->limits.y.min = -1.0;
		this->limits.y.max = 1.0;
		this->limits.z.min = -1.0;
		this->limits.z.max = 1.0;
	}

	PointCloudFilter(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
	{
		this->limits.x.min = minX;
		this->limits.x.max = maxX;
		this->limits.y.min = minY;
		this->limits.y.max = maxY;
		this->limits.z.min = minZ;
		this->limits.z.max = maxZ;
	}

	virtual ~PointCloudFilter(){}

	template <class _type> int cutOffFilter(pcl::PointCloud<_type>& pointsIn, pcl::PointCloud<_type>& pointsOut)
	{
		pcl::PointCloud<_type> aux1, aux2;
		pcl::PassThrough<_type> pass;

		//	Set x-limits
		pass.setInputCloud(pointsIn.makeShared());
		pass.setFilterFieldName("x");
		pass.setFilterLimits(limits.x.min,limits.x.max);
		pass.filter(aux1);

		//	Set y-limits
		pass.setInputCloud(aux1.makeShared());
		pass.setFilterFieldName("y");
		pass.setFilterLimits(limits.y.min,limits.y.max);
		pass.filter(aux2);

		//	Set z-limits
		pass.setInputCloud(aux2.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(limits.z.min,limits.z.max);
		pass.filter(pointsOut);

		return 0;
	}
};

#endif /* POINTCLOUDFILTER_HPP_ */
