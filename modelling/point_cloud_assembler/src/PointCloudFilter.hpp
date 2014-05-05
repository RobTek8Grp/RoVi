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
public:
	PointCloudFilter(){}
	virtual ~PointCloudFilter(){}

	template <class _type> int cutOffFilter(pcl::PointCloud<_type>& pointsIn, pcl::PointCloud<_type>& pointsOut)
	{
		pcl::PointCloud<_type> aux1, aux2;
		pcl::PassThrough<_type> pass;

		//	Set x-limits
		pass.setInputCloud(pointsIn.makeShared());
		pass.setFilterFieldName("x");
		pass.setFilterLimits(-.2,.2);
		pass.filter(aux1);

		//	Set y-limits
		pass.setInputCloud(aux1.makeShared());
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-.2,.2);
		pass.filter(aux2);

		//	Set z-limits
		pass.setInputCloud(aux2.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(.5,1.0);
		pass.filter(pointsOut);

		return 0;
	}
};

#endif /* POINTCLOUDFILTER_HPP_ */
