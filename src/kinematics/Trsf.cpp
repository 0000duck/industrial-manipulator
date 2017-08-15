/*
 * Trsf.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: a1994846931931
 */

# include "Trsf.h"
# include "../math/HTransform3D.h"
# include <math.h>

namespace robot {
namespace kinematic {

Trsf::Trsf() {
	_x = _y = _z = _rx = _ry = _rz = 0;
	_isTranformChanged = false;
}

Trsf::Trsf(double x, double y, double z, double rx, double ry, double rz):
		_x(x),
		_y(y),
		_z(z),
		_rx(rx),
		_ry(ry),
		_rz(rz){
	_isTranformChanged = true;
}

robot::math::HTransform3D<double>& Trsf::getTransform(){
	if (_isTranformChanged)
		this->doGetTransform();
	return _tran;
}

void Trsf::doGetTransform(){
	//TODO return the homogeneous transform(HTransform) of the Trsf
	const double s1 = sin(_rx);	const double c1 = cos(_rx);
	const double s2 = sin(_ry);	const double c2 = cos(_ry);
	const double s3 = sin(_rz);	const double c3 = cos(_rz);
	static robot::math::Rotation3D<double> rot(
			c2*c3, -c2*s3, s2,
			s1*s2*c3 + c1*s3, -s1*s2*s3 + c1*c3, -s1*c2,
			-c1*s2*c3 + s1*s3, c1*s2*s3 + s1*c3, c1*c2);
	static robot::math::Vector3D<double> vec(_x, _y, _z);
	static robot::math::HTransform3D<double> tran(vec, rot);
	_tran = tran;
}

Trsf::~Trsf() {
	// TODO Auto-generated destructor stub
}

} /* namespace kinematic */
} /* namespace robot */
