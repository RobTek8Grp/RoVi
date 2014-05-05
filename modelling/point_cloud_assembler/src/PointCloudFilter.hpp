/*
 * PointCloudFilter.hpp
 *
 *  Created on: May 4, 2014
 *      Author: kent
 */

#ifndef POINTCLOUDFILTER_HPP_
#define POINTCLOUDFILTER_HPP_

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

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
	} cutOffLimits;

	struct
	{
		double x;
		double y;
		double z;
	} voxelLeafSizes;

public:
	PointCloudFilter()
	{
		//	Cut-Off filter limits
		this->cutOffLimits.x.min = -1.0;
		this->cutOffLimits.x.max = 1.0;
		this->cutOffLimits.y.min = -1.0;
		this->cutOffLimits.y.max = 1.0;
		this->cutOffLimits.z.min = -1.0;
		this->cutOffLimits.z.max = 1.0;

		//	Voxel filter leaf sizes
	}

	virtual ~PointCloudFilter(){}

	void setCutOffLimits(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
	{
		this->cutOffLimits.x.min = minX;
		this->cutOffLimits.x.max = maxX;
		this->cutOffLimits.y.min = minY;
		this->cutOffLimits.y.max = maxY;
		this->cutOffLimits.z.min = minZ;
		this->cutOffLimits.z.max = maxZ;
	}

	template <class _type> int cutOffFilter(pcl::PointCloud<_type>& pointsIn, pcl::PointCloud<_type>& pointsOut)
	{
		pcl::PointCloud<_type> aux1, aux2;
		pcl::PassThrough<_type> pass;

		//	Set x-limits
		pass.setInputCloud(pointsIn.makeShared());
		pass.setFilterFieldName("x");
		pass.setFilterLimits(cutOffLimits.x.min,cutOffLimits.x.max);
		pass.filter(aux1);

		//	Set y-limits
		pass.setInputCloud(aux1.makeShared());
		pass.setFilterFieldName("y");
		pass.setFilterLimits(cutOffLimits.y.min,cutOffLimits.y.max);
		pass.filter(aux2);

		//	Set z-limits
		pass.setInputCloud(aux2.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(cutOffLimits.z.min,cutOffLimits.z.max);
		pass.filter(pointsOut);

		return 0;
	}

	void setVoxelLeafSizes(double x, double y, double z)
	{
		this->voxelLeafSizes.x = x;
		this->voxelLeafSizes.y = y;
		this->voxelLeafSizes.z = z;
	}

	template <class _type> int voxelFilter(pcl::PointCloud<_type>& pointsIn, pcl::PointCloud<_type>& pointsOut)
	{
		pcl::VoxelGrid<_type> grid;

		grid.setLeafSize(this->voxelLeafSizes.x, this->voxelLeafSizes.y, this->voxelLeafSizes.z);
		grid.setInputCloud(pointsIn.makeShared());
		grid.filter(pointsOut);
	}
};

#endif /* POINTCLOUDFILTER_HPP_ */
