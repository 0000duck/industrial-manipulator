/*
 * RotationInterpolator.cpp
 *
 *  Created on: Oct 23, 2017
 *      Author: a1994846931931
 */

#include "RotationInterpolator.h"
# include "../math/Quaternion.h"

using robot::math::Quaternion;

namespace robot {
namespace trajectory {

RotationInterpolator::RotationInterpolator(Rotation3D<double>& start, Vector3D<double>& n, double rad, double duration)
: _start(start),
  _n(n),
  _rad(rad)
{
	if (n == Vector3D<double>())
		rad = 0;
	else
		_n.doNormalize();

	if (duration == 0)
	{
		if (rad != 0)
			_duration = rad;
		else
			_duration = 0.001;
	}
	else if (duration < 0)
		throw("错误<RotationInterpolator>: 时长不能为负数!");
	else
		_duration = duration;

}

Rotation3D<double> RotationInterpolator::x(double t) const
{
	double drad = _rad*t/_duration;
	return _start*(Quaternion(drad, _n).toRotation3D());
}

Rotation3D<double> RotationInterpolator::dx(double t) const
{

}

Rotation3D<double> RotationInterpolator::ddx(double t) const
{

}

double RotationInterpolator::duration() const
{
	return _duration;
}

RotationInterpolator::~RotationInterpolator()
{

}

} /* namespace trajectory */
} /* namespace robot */
