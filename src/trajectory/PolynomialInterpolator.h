/**
 * @brief PolynomialInterpolator类
 * @date Sep 4, 2017
 * @author a1994846931931
 */

#ifndef POLYNOMIALINTERPOLATOR_H_
#define POLYNOMIALINTERPOLATOR_H_

#include "Interpolator.h"
# include <vector>
# include "../math/Quaternion.h"
# include "../math/Rotation3D.h"
# include "../math/HTransform3D.h"
# include "../math/Quaternion.h"
# include "../math/Q.h"
namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */
template <class T>
class PolynomialInterpolator: public Interpolator<T> {

public:
	PolynomialInterpolator(const T& a,const T& b,const T& c,double duration):
		_a(a),_b(b),_c(c),_vel(b+c*duration),_acc(c),_duration(duration)
{
}

	virtual ~PolynomialInterpolator(){}
	T x(double t) const
	{
		return _a+(_b+_c*t)*t;
	}
	T dx(double t)const
	{
		return _b+_c*t;
	}
	T ddx()const
	{
		return _c;
	}

private:
	T _a;
	T _b;
	T _c;
	T _vel;
	T _acc;
	double _duration;
};


template <class T>
class PolynomialInterpolator3: public Interpolator<T> {

public:
	PolynomialInterpolator3(const T& a,const T& b,const T& c,const T&d,double duration):
		_a(a),_b(b),_c(c),_d(d),_vel(b+c*duration),_acc(c),_duration(duration)
{
}

	virtual ~PolynomialInterpolator3(){}
	T x(double t) const
	{
		return _a+(_b+(_c+_d*t)*t)*t;
	}
	T dx(double t)const
	{
		return _b+(_c+_d*t)*t;
	}
	T ddx(double t)const
	{
		return _c+_d*t;
	}

private:
	T _a;
	T _b;
	T _c;
	T _d;
	T _vel;
	T _acc;
	double _duration;
};







/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* POLYNOMIALINTERPOLATOR_H_ */
