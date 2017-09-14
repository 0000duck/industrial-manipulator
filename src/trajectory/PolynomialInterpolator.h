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
# include <memory>
namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */
template <class T>
class PolynomialInterpolator2: public Interpolator<T> {
public:
	using ptr = std::shared_ptr<PolynomialInterpolator2>;
	PolynomialInterpolator2(const T& a,const T& b,const T& c,double duration):
		_a(a),_b(b),_c(c),_vel(b+2*c*duration),_acc(2*c),_duration(duration)
{
}

	virtual ~PolynomialInterpolator2(){}
	T x(double t) const
	{
		return _a+(_b+_c*t)*t;
	}
	T dx(double t)const
	{
		return _b+2*_c*t;
	}
	T ddx(double t)const
	{
		return 2*_c;
	}

	double duration() const
	{
		return _duration;
	}

	static PolynomialInterpolator2::ptr make(double x1,double x2,double x3,double y1,double y2,double y3,double duration)
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

			return PolynomialInterpolator2::ptr(new PolynomialInterpolator2(z(0,0), z(1,0), z(2,0), duration));
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
	using ptr = std::shared_ptr<PolynomialInterpolator3>;
	PolynomialInterpolator3(const T& a,const T& b,const T& c,const T&d,double duration):
		_a(a),_b(b),_c(c),_d(d),_vel(_b+(2*_c+3*_d*duration)*duration),_acc(2*_c+6*_d*duration),_duration(duration)
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
	static PolynomialInterpolator3::ptr make(double x1,double x2,double x3,double x4,double y1,double y2,double y3,double y4,double duration)
	{
		Eigen::MatrixXd j=(4,4);
		j(0,0)=1;j(0,1)=x1;j(0,2)=x1*x1;j(0,3)=j(0,2)*x1;
		j(1,0)=1;j(1,1)=x2;j(1,2)=x2*x2;j(1,3)=j(1,2)*x2;
		j(2,0)=1;j(2,1)=x3;j(2,2)=x3*x3;j(2,3)=j(2,2)*x3;
		j(3,0)=1;j(3,1)=x4;j(3,2)=x4*x4;j(3,3)=j(3,2)*x4;
		j.inverse();
		Eigen::MatrixXd v=(4,1);
		v<<y1,y2,y3,y4;
		Eigen::MatrixXd z=(4,1);
		z=j*v;

		return PolynomialInterpolator3::ptr(new PolynomialInterpolator3(z(0,0), z(1,0), z(2,0), z(3,0), duration));
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

template <class T>
class PolynomialInterpolator4: public Interpolator<T> {
public:
	using ptr = std::shared_ptr<PolynomialInterpolator4>;
	PolynomialInterpolator4(const T& a,const T& b,const T& c,const T& d,const T& e,double duration):
		_a(a),_b(b),_c(c),_d(d),_e(e),_vel(b+(2*c+(3*d+4*e*duration)*duration)*duration),_acc(2*c+(6*d+12*e*duration)*duration),_duration(duration)
{
}

	virtual ~PolynomialInterpolator4(){}
	T x(double t) const
	{
		return _a+(_b+(_c+(_d+_e*t)*t)*t)*t;
	}
	T dx(double t)const
	{
		return _b+(2*_c+(3*_d+4*_e*t)*t)*t;
	}
	T ddx(double t)const
	{
		return 2*_c+(6*_d+12*_e*t)*t;
	}

	double duration() const
	{
		return _duration;
	}

	static PolynomialInterpolator4::ptr make(double x1,double x2,double x3,double x4,double x5,double y1,double y2,double y3,double y4,double y5,double duration)
		{
			Eigen::MatrixXd j=(5,5);
			j(0,0)=1;j(0,1)=x1;j(0,2)=x1*x1;j(0,3)=j(0,2)*x1;j(0,4)=j(0,3)*x1;
			j(1,0)=1;j(1,1)=x2;j(1,2)=x2*x2;j(1,3)=j(1,2)*x2;j(1,4)=j(1,3)*x2;
			j(2,0)=1;j(2,1)=x3;j(2,2)=x3*x3;j(2,3)=j(2,2)*x3;j(2,4)=j(2,3)*x3;
			j(3,0)=1;j(3,1)=x4;j(3,2)=x4*x4;j(3,3)=j(3,2)*x4;j(3,4)=j(3,3)*x4;
			j(4,0)=1;j(4,1)=x5;j(4,2)=x5*x5;j(4,3)=j(4,2)*x5;j(4,4)=j(4,3)*x5;
			j.inverse();
			Eigen::MatrixXd v=(5,1);
			v<<y1,y2,y3,y4,y5;
			Eigen::MatrixXd z=(5,1);
			z=j*v;

			return PolynomialInterpolator4::ptr(new PolynomialInterpolator4(z(0,0), z(1,0), z(2,0),z(3,0),z(4,0), duration));
		}
private:
	T _a;
	T _b;
	T _c;
	T _d;
	T _e;
	T _vel;
	T _acc;
	double _duration;
};

template <class T>
class PolynomialInterpolator5: public Interpolator<T> {
public:
	using ptr = std::shared_ptr<PolynomialInterpolator5>;
	PolynomialInterpolator5(const T& a,const T& b,const T& c,const T& d,const T& e,const T& f,double duration):
		_a(a),_b(b),_c(c),_d(d),_e(e),_f(f),_vel(b+(2*c+(3*d+(4*e+5*f*duration)*duration)*duration)*duration),_acc(2*c+(6*d+(12*e+20*f*duration)*duration)*duration),_duration(duration)
{
}

	virtual ~PolynomialInterpolator5(){}
	T x(double t) const
	{
		return _a+(_b+(_c+(_d+(_e+_f*t)*t)*t)*t)*t;
	}
	T dx(double t)const
	{
		return _b+(2*_c+(3*_d+(4*_e+5*_f*t)*t)*t)*t;
	}
	T ddx(double t)const
	{
		return 2*_c+(6*_d+(12*_e+20*_f*t)*t)*t;
	}

	double duration() const
	{
		return _duration;
	}

	static PolynomialInterpolator5::ptr make(double x1,double x2,double x3,double x4,double x5,double x6,double y1,double y2,double y3,double y4,double y5,double y6,double duration)
		{
			Eigen::MatrixXd j=(6,6);
			j(0,0)=1;j(0,1)=x1;j(0,2)=x1*x1;j(0,3)=j(0,2)*x1;j(0,4)=j(0,3)*x1;j(0,5)=j(0,4)*x1;
			j(1,0)=1;j(1,1)=x2;j(1,2)=x2*x2;j(1,3)=j(1,2)*x2;j(1,4)=j(1,3)*x2;j(1,5)=j(1,4)*x2;
			j(2,0)=1;j(2,1)=x3;j(2,2)=x3*x3;j(2,3)=j(2,2)*x3;j(2,4)=j(2,3)*x3;j(2,5)=j(2,4)*x3;
			j(3,0)=1;j(3,1)=x4;j(3,2)=x4*x4;j(3,3)=j(3,2)*x4;j(3,4)=j(3,3)*x4;j(3,5)=j(3,4)*x4;
			j(4,0)=1;j(4,1)=x5;j(4,2)=x5*x5;j(4,3)=j(4,2)*x5;j(4,4)=j(4,3)*x5;j(4,5)=j(4,4)*x5;
			j(5,0)=1;j(5,1)=x6;j(5,2)=x6*x6;j(5,3)=j(5,2)*x6;j(5,4)=j(5,3)*x6;j(5,5)=j(5,4)*x6;
			j.inverse();
			Eigen::MatrixXd v=(6,1);
			v<<y1,y2,y3,y4,y5,y6;
			Eigen::MatrixXd z=(6,1);
			z=j*v;

			return PolynomialInterpolator5::ptr(new PolynomialInterpolator5(z(0,0), z(1,0), z(2,0),z(3,0),z(4,0), duration));
		}

	/**
	 * @brief 根据t1, t2时间的x, v, a构造五次多项式插补器
	 * @param t1 [in] t1时刻
	 * @param t2 [in] t2时刻
	 * @param x1 [in] t1时刻的位置
	 * @param v1 [in] t1时刻的速度
	 * @param a1 [in] t1时刻的加速度
	 * @param x2 [in] t2时刻的位置
	 * @param v2 [in] t2时刻的速度
	 * @param a2 [in] t2时刻的加速度
	 * @param duration [in] 插补时长
	 * @return 指向构造的五次多项式插补器的shared_ptr
	 */
	static PolynomialInterpolator5::ptr make(double t1, double t2, double x1, double v1, double a1, double x2, double v2, double a2, double duration)
	{
		Eigen::MatrixXd j(6, 6);
		Eigen::MatrixXd v(6, 1);
		Eigen::MatrixXd z(6, 1);
		double t12 = t1*t1;
		double t13 = t12*t1;
		double t14 = t13*t1;
		double t15 = t14*t1;
		double t22 = t2*t2;
		double t23 = t22*t2;
		double t24 = t23*t2;
		double t25 = t24*t2;
		j(0,0)=1; 	j(0,1)=t1; 	j(0,2)=t12;		j(0,3)=t13;		j(0,4)=t14;		j(0,5)=t15;
		j(1,0)=0; 	j(1,1)=1; 	j(1,2)=2*t1;	j(1,3)=3*t12;	j(1,4)=4*t13;	j(1,5)=5*t14;
		j(2,0)=0; 	j(2,1)=0; 	j(2,2)=2;		j(2,3)=6*t1;	j(2,4)=12*t12;	j(2,5)=20*t13;
		j(3,0)=1; 	j(3,1)=t2; 	j(3,2)=t22;		j(3,3)=t23;		j(3,4)=t24;		j(3,5)=t25;
		j(4,0)=0; 	j(4,1)=1;  	j(4,2)=2*t2;	j(4,3)=3*t22;	j(4,4)=4*t23;	j(4,5)=5*t24;
		j(5,0)=0; 	j(5,1)=0;  	j(5,2)=2;		j(5,3)=6*t2;	j(5,4)=12*t22;	j(5,5)=20*t23;
		v << x1, v1, a1, x2, v2, a2;
		z = j.inverse()*v;
		return PolynomialInterpolator5::ptr(new PolynomialInterpolator5(z(0,0), z(1,0), z(2,0), z(3,0), z(4,0), z(5,0), duration));
	}
private:
	T _a;
	T _b;
	T _c;
	T _d;
	T _e;
	T _f;
	T _vel;
	T _acc;
	double _duration;
};




/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* POLYNOMIALINTERPOLATOR_H_ */
