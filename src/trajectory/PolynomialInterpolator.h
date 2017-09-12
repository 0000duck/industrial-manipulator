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
# include "../ext/Eigen/Dense"
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
		return _b+2*_c*t;
	}
	T ddx()const
	{
		return 2*_c;
	}

	double duration() const
	{
		return _duration;
	}

	static PolynomialInterpolator make(double x1,double x2,double x3,double y1,double y2,double y3,double duration)
		{
			Eigen::MatrixXd j=(3,3);
			j(0,0)=1;j(0,1)=x1;j(0,2)=x1*x1;
			j(1,0)=1;j(1,1)=x2;j(1,2)=x2*x2;
			j(2,0)=1;j(2,1)=x3;j(2,2)=x3*x3;
			j.inverse();
			Eigen::MatrixXd v=(3,1);
			v<<y1,y2,y3;
			Eigen::MatrixXd z=(3,1);
			z=j*v;

			return PolynomialInterpolator (_a=z(0,0),_b=z(1,0),_c=z(2,0),_duration=duration);
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
		return _b+(2*_c+3*_d*t)*t;
	}

	T ddx(double t)const
	{
		return 2*_c+6*_d*t;
	}
	double duration() const
	{
		return _duration;
	}
	static PolynomialInterpolator3 make(double x1,double x2,double x3,double x4,double y1,double y2,double y3,double y4,double duration)
	{
		Eigen::MatrixXd j=(4,4);
		j(0,0)=1;j(0,1)=x1;j(0,2)=x1*x1;
		j(1,0)=1;j(1,1)=x2;j(1,2)=x2*x2;
		j(2,0)=1;j(2,1)=x3;j(2,2)=x3*x3;
		j(3,0)=1;j(3,1)=x4;j(3,2)=x4*x4;
		j.inverse();
		Eigen::MatrixXd v=(4,1);
		v<<y1,y2,y3,y4;
		Eigen::MatrixXd z=(4,1);
		z=j*v;

		return PolynomialInterpolator3(_a=z(0,0),_b=z(1,0),_c=z(2,0),_d=z(3,0),_duration=duration);
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
