/*
 * Trsf.h
 *
 *  Created on: Aug 11, 2017
 *      Author: a1994846931931
 */

#ifndef TRSF_H_
#define TRSF_H_

# include "../math/HTransform3D.h"

using namespace robot::math;

namespace robot {
namespace kinematic {

class Trsf {
public:
	Trsf();
	Trsf(double x, double y, double z, double rx, double ry, double rz);
	robot::math::HTransform3D<double> getTransform();
	virtual ~Trsf();
private:
	double _x;
	double _y;
	double _z;
	double _rx;
	double _ry;
	double _rz;
	robot::math::HTransform3D<double> _tran;
	bool _isTranformChanged;
private:
	void doGetTransform();
};

} /* namespace kinematic */
} /* namespace robot */

#endif /* TRSF_H_ */
