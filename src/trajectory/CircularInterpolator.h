/**
 * @brief 圆弧插补器
 * @date Sep 4, 2017
 * @author a1994846931931
 */

#ifndef CIRCULARINTERPOLATOR_H_
#define CIRCULARINTERPOLATOR_H_

# include "Interpolator.h"
# include "../math/HTransform3D.h"

using namespace robot::math;

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */

/**
 * @brief 圆弧插补器模板类
 *
 * 可以由三个点构造一个圆弧插补器, 根据时间参数给出当前的位置, 速度和加速度.
 */
template <class T>
class CircularInterpolator: public Interpolator<T> {
public:
	virtual ~CircularInterpolator(){}
};

/**
 * @brief Vector3D<T>类的圆弧插补器.
 *
 */
template <class T>
class CircularInterpolator<Vector3D<T> >: public Interpolator<Vector3D<T> > {
public:
	CircularInterpolator(
			const Vector3D<T>& p1,
			const Vector3D<T>& p2,
			const Vector3D<T>& p3,
			double duration):
				_p1(p1),
				_p2(p2),
				_p3(p3),
				_duration(duration)
	{
        Vector3D<T> p12Xp13 = Vector3D<T>::cross(p2-p1, p3-p1);
        const double p12Xp13Length = p12Xp13.getLengh();

        if (fabs(p12Xp13Length) < 1e-15)
            throw(
                "Unable to make circular interpolator "
                "based on three points on a straight line");

        Vector3D<T> nz = p12Xp13 / p12Xp13Length;
        Vector3D<T> nx = Vector3D<T>::normalize(p2-p1);
        Vector3D<T> ny = Vector3D<T>::cross(nz, nx);
        _T = HTransform3D<T>(p1, Rotation3D<T>(nx, ny, nz));

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

	Vector3D<T> x(double t) const
	{
        const double tau = (_tend-_tstart)/_duration*t + _tstart;
        Vector3D<T> v;
        v(0) = _r*cos(tau)+_cx;
        v(1) = _r*sin(tau)+_cy;
        v(2) = 0;
        return _T*v;
	}

	Vector3D<T> dx(double t) const
	{
        const double a = (_tend-_tstart)/_duration;
        const double tau = a*t + _tstart;

        Vector3D<double> v;
        v(0) = -_r*a*sin(tau);
        v(1) = _r*a*cos(tau);
        v(2) = 0;
        return _T.getRotation()*v;
	}

    Vector3D<T> ddx(double t) const
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
	Vector3D<T> _p1;
	Vector3D<T> _p2;
	Vector3D<T> _p3;
	double _duration;
	HTransform3D<T> _T;
	double _cx;
	double _cy;
	double _r;
	double _tstart;
	double _tend;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* CIRCULARINTERPOLATOR_H_ */
