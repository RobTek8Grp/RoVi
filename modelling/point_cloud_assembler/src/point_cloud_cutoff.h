/*
	Created on:	April 1st, 2014		(Aprils fool)
	Author:		Kent Stark Olsen
	E-Mail:		kent.stark.olsen@gmail.com
*/

#ifndef POINTCLOUDFILTER_H_
#define POINTCLOUDFILTER_H_

#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

template <class _type> class PointCloudCutoff
{
	pcl::PointCloud<_type> output;
	pcl::PassThrough<_type> passThroughFilter;

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

		bool active;
	} cloudLimits;

public:
	PointCloudCutoff();
	PointCloudCutoff(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
	virtual ~PointCloudCutoff();

	pcl::PointCloud<_type>::Ptr filterCloud (_type::Ptr input);
	bool toggleActivation(void);
};

PointCloudCutoff::PointCloudCutoff()
{
	this->output = new pcl::PointCloud<_type>();

	this->cloudLimits.active = false;
	this->cloudLimits.x.min = -1.0;
	this->cloudLimits.x.max = 1.0;
	this->cloudLimits.y.min = -1.0;
	this->cloudLimits.y.max = 1.0;
	this->cloudLimits.z.min = -1.0;
	this->cloudLimits.z.max = 1.0;
}

PointCloudCutoff::PointCloudCutoff(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
	this->cloudLimits.active = true;
	this->cloudLimits.x.min = minX;
	this->cloudLimits.x.max = maxX;
	this->cloudLimits.y.min = minY;
	this->cloudLimits.y.max = maxY;
	this->cloudLimits.z.min = minZ;
	this->cloudLimits.z.max = maxZ;
}

PointCloudCutoff::~PointCloudCutoff()
{
}

_type PointCloudCutoff::filterCloud(_type::Ptr input)
{
	//	Pass through filter (x)
	this->passThroughFilter.setInputCloud(input);
	this->passThroughFilter.setFilterFieldName("x");
	this->passThroughFilter.setFilterLimits (this->cloudLimits.x.min, this->cloudLimits.x.max);
	this->passThroughFilter.filter(*)
}

bool PointCloudCutoff::toggleActivation(void)
{
	this->cloudLimits.active = !this->cloudLimits.active;
}

#endif	/* POINTCLOUDFILTER_H_ */
