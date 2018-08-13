/*
 * TwopartBezier.cpp
 *
 *  Created on: Oct 13, 2017
 *      Author: zrf
 */

# include "TwopartBezier.h"
# include "../math/Integrator.h"
# include <algorithm>
# include "../trajectory/BezierPath.h"
# include "math.h"
# include "../trajectory/SequenceInterpolator.h"
# include "../trajectory/Interpolator.h"
# include "../math/Q.h"

using robot::math::Integrator;

namespace robot {
namespace trajectory {

using namespace robot::trajectory;

//TwopartBezier::TwopartBezier() {
//	// TODO Auto-generated constructor stub
//
//}

TwopartBezier::TwopartBezier(const point &p1, const point &p2, const point &p3, double k, double duration)
:_p1(p1), _p2(p2), _p3(p3), _duration(duration)
{
	if(k < 0||k >0.5)
	{
		throw("k应为0~0.5的值");
	}
	point p12 = _p2 - _p1;
	point p23 = _p3 - _p2;
	double theta = acos(point::dot(p12, p23)/p12.getLength()/p23.getLength());
	double alpha = (180 - theta)/2;
	_b11 = p12*0.5 + _p1;
	_b12 = p12*(1 - k) + _p1;
	_b21 = p23*0.5 + _p2;
	_b22 = p23*k + _p2;
	point qplan = _b22 - _b12;
	_q = qplan*0.5;
	BezierPath::ptr part1(new BezierPath(_p1, _b11, _b12, _q));
	BezierPath::ptr part2(new BezierPath(_q, _b22, _b21, _p2));
	_part -> addInterpolator (part1);
	_part -> addInterpolator (part2);
}

robot::math::Vector3D<double> TwopartBezier::x(double t)
{
	return _part -> x(t);
}

robot::math::Vector3D<double> TwopartBezier::dx(double t)
{
	return _part -> dx(t);
}

robot::math::Vector3D<double> TwopartBezier::ddx(double t)
{
	return _part -> ddx(t);
}

TwopartBezier::~TwopartBezier() {
	// TODO Auto-generated destructor stub
}

} /* namespace trajectory */
} /* namespace robot */
