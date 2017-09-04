/*
 * CircularInterpolator.h
 *
 *  Created on: Sep 4, 2017
 *      Author: a1994846931931
 */

#ifndef CIRCULARINTERPOLATOR_H_
#define CIRCULARINTERPOLATOR_H_

# include "Interpolator.h"
# include "../math/HTransform3D.h"

using namespace robot::math;

namespace robot {
namespace trajectory {

template <class T>
class CircularInterpolator<T>: public Interpolator<T> {
public:
	virtual ~CircularInterpolator(){}
};

class CircularInterpolator<Vector3D<double> >: public Interpolator<Vector3D<double> > {
public:
	CircularInterpolator(
			const Vector3D<double>& p1,
			const Vector3D<double>& p2,
			const Vector3D<double>& p3,
			double duration):
				_p1(p1),
				_p2(p2),
				_p3(p3),
				_duration(duration)
	{
        Vector3D<double> p12Xp13 = Vector3D<double>::cross(p2-p1, p3-p1);
        const double p12Xp13Length = p12Xp13.getLengh();

        if (fabs(p12Xp13Length) < 1e-15)
            throw(
                "Unable to make circular interpolator "
                "based on three points on a straight line");

        Vector3D<double> nz = p12Xp13 / p12Xp13Length;
        Vector3D<double> nx = Vector3D<double>::normalize(p2-p1);
        Vector3D<double> ny = Vector3D<double>::cross(nz, nx);
        _T = HTransform3D<double>(p1, Rotation3D<double>(nx, ny, nz));

        const double x2 = (p2-p1).getLengh();
        const double p3p1Length = (p3-p1).getLengh();
        const double theta = asin(p12Xp13Length/(x2 * p3p1Length));
        const double x3 = cos(theta)*p3p1Length;
        const double y3 = sin(theta)*p3p1Length;

        _r = sqrt((x2*x2 + (-(x2*x3) + x3*x3 + y3*y3)*(-(x2*x3) + x3*x3 + y3*y3) / y3*y3)) / 2;
        _cx = x2 / 2.0;
        _cy = (-x2 * x3 + x3*x3 + y3*y3) / (2. * y3);
        _tstart = atan2(-_cy/_r, -_cx/_r);
        _tend = atan2((y3-_cy)/_r, (x3-_cx)/_r);
        if (_tend <= _tstart)
        	_tend += 2*M_PI;
	}

	Vector3D<double> x(double t) const
	{
        const double tau = (_tend-_tstart)/_duration*t + _tstart;
        Vector3D<double> v;
        v(0) = _r*cos(tau)+_cx;
        v(1) = _r*sin(tau)+_cy;
        v(2) = 0;
        return _T*v;
	}

	Vector3D<double> dx(double t) const
	{
        const double a = (_tend-_tstart)/_duration;
        const double tau = a*t + _tstart;

        Vector3D<double> v;
        v(0) = -_r*a*sin(tau);
        v(1) = _r*a*cos(tau);
        v(2) = 0;
        return _T.getRotation()*v;
	}

    Vector3D<double> ddx(double t) const
    {
        const double a = (_tend-_tstart)/_duration;
        const double tau = a*t + _tstart;

        Vector3D<double> v;
        v(0) = -_r*a*a*cos(tau);
        v(1) = -_r*a*a*sin(tau);
        v(2) = 0;
        return _T.getRotation()()*v;
    }

	virtual ~CircularInterpolator(){}
private:
	Vector3D<double> _p1;
	Vector3D<double> _p2;
	Vector3D<double> _p3;
	double _duration;
	HTransform3D<double> _T;
	double _cx;
	double _cy;
	double _r;
	double _tstart;
	double _tend;
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* CIRCULARINTERPOLATOR_H_ */
