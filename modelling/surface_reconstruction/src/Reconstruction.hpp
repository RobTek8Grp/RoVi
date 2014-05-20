/*
 * Reconstruction.hpp
 *
 *  Created on: May 16, 2014
 *      Author: kent
 */

#ifndef RECONSTRUCTION_HPP_
#define RECONSTRUCTION_HPP_


struct DataPoint
{
	double x;
	double y;
	double z;

	double dx;
	double dy;
	double dz;

	DataPoint()
	{
		this->x = 0;
		this->y = 0;
		this->z = 0;

		this->dx = 0;
		this->dy = 0;
		this->dz = 1;
	}

	DataPoint(double x, double y, double z, double dx, double dy, double dz)
	{
		this->x = x;
		this->y = y;
		this->z = z;

		this->dx = dx;
		this->dy = dy;
		this->dz = dz;
	}
};

class Reconstruction
{
public:
	Reconstruction();
	virtual ~Reconstruction();

	int startReconstruction(std::vector<DataPoint> input);

};

#endif /* RECONSTRUCTION_HPP_ */
