/**
 * @brief 圆弧插补器
 * @date Sep 4, 2017
 * @author a1994846931931
 */

#ifndef CIRCULARINTERPOLATOR_H_
#define CIRCULARINTERPOLATOR_H_

# include "Interpolator.h"
# include "../math/HTransform3D.h"
# include <memory>

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
 * 三维向量可以定义一个空间点, 三个不共线的空间点可以描述一个空间圆弧.
 */
template <class T>
class CircularInterpolator<Vector3D<T> >: public Interpolator<Vector3D<T> > {
public:
	using ptr = std::shared_ptr<CircularInterpolator<Vector3D<T> > >;

	/**
	 * @brief 构造函数
	 * @param p1 [in] 开始点
	 * @param p2 [in] 中间点
	 * @param p3 [in] 结束点
	 * @param duration [in] 插补时间
	 */
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
            throw("错误<CircularInterpolator>: 无法通过一条直线上的三个点来构造圆弧");

        Vector3D<T> nz = p12Xp13 / p12Xp13Length;
        Vector3D<T> nx = Vector3D<T>::normalize(p2-p1);
        Vector3D<T> ny = Vector3D<T>::cross(nz, nx);
        _T = HTransform3D<T>(p1, Rotation3D<T>(nx, ny, nz));

        const double x2 = (p2-p1).getLengh();
        const double p3p1Length = (p3-p1).getLengh();
        const double theta = asin(p12Xp13Length/(x2 * p3p1Length));

        const double x3 = cos(theta)*p3p1Length;
        const double y3 = sin(theta)*p3p1Length;
//
//        _r = sqrt((x2*x2 + (-(x2*x3) + x3*x3 + y3*y3)*(-(x2*x3) + x3*x3 + y3*y3) / y3*y3)) / 2;
//        _cx = x2 / 2.0;
//        _cy = (-x2 * x3 + x3*x3 + y3*y3) / (2. * y3);

       double l12 = x2;
       double l23 = (p2 - p3).getLength();
       double l13 = (p1 - p3).getLength();
       double theta2 = acos((l23*l23 + l13*l13 - l12*l12)/(2*l13*l23));
       _r = l12/(2. * sin(theta2));
       _cx = x2/2.0;
       _cy = _r*cos(theta2);


        _tstart = atan2(-_cy/_r, -_cx/_r);
        _tend = atan2((y3-_cy)/_r, (x3-_cx)/_r);
//        if (_tend <= _tstart)
//        	_tend += 2*M_PI;
        if (_tend <= _tstart)
        	throw("错误<CircularInterpolator>: 不正确的初始点位置, 无法规划");
	}

	/**
	 * @brief 构造函数 duration等于圆弧长度
	 * @param p1 [in] 开始点
	 * @param p2 [in] 中间点
	 * @param p3 [in] 结束点
	 */
	CircularInterpolator(
			const Vector3D<T>& p1,
			const Vector3D<T>& p2,
			const Vector3D<T>& p3):
				_p1(p1),
				_p2(p2),
				_p3(p3)
	{
        Vector3D<T> p12Xp13 = Vector3D<T>::cross(p2-p1, p3-p1);
        const double p12Xp13Length = p12Xp13.getLength();

        if (fabs(p12Xp13Length) < 1e-15)
            throw("错误<CircularInterpolator>: 无法通过一条直线上的三个点来构造圆弧");

        Vector3D<T> nz = p12Xp13 / p12Xp13Length;
        Vector3D<T> nx = Vector3D<T>::normalize(p2-p1);
        Vector3D<T> ny = Vector3D<T>::cross(nz, nx);
        _T = HTransform3D<T>(p1, Rotation3D<T>(nx, ny, nz));

        const double x2 = (p2-p1).getLength();
        const double p3p1Length = (p3-p1).getLength();
        const double theta = asin(p12Xp13Length/(x2 * p3p1Length));
        const double x3 = cos(theta)*p3p1Length;
        const double y3 = sin(theta)*p3p1Length;

//        _r = sqrt((x2*x2 + (-(x2*x3) + x3*x3 + y3*y3)*(-(x2*x3) + x3*x3 + y3*y3) / y3*y3)) / 2;
//        _cx = x2 / 2.0;
//        _cy = (-x2 * x3 + x3*x3 + y3*y3) / (2. * y3);

        double l12 = x2;
        double l23 = (p2 - p3).getLength();
        double l13 = (p1 - p3).getLength();
        double theta2 = acos((l23*l23 + l13*l13 - l12*l12)/(2*l13*l23));
        _r = l12/(2. * sin(theta2));
        _cx = x2/2.0;
        _cy = _r*cos(theta2);


        _tstart = atan2(-_cy/_r, -_cx/_r);
        _tend = atan2((y3-_cy)/_r, (x3-_cx)/_r);
//        if (_tend <= _tstart)
//        	_tend += 2*M_PI;
        if (_tend <= _tstart)
        	throw("错误<CircularInterpolator>: 不正确的初始点位置, 无法规划");
        _duration =  getLength();
	}

	Vector3D<T> x(double t) const
	{
        const double tau = (_tend-_tstart)/_duration*t + _tstart;
        Vector3D<T> v;
        v[0] = _r*cos(tau)+_cx;
        v[1] = _r*sin(tau)+_cy;
        v[2] = 0;
        return _T*v;
	}

	Vector3D<T> dx(double t) const
	{
        const double a = (_tend-_tstart)/_duration;
        const double tau = a*t + _tstart;

        Vector3D<double> v;
        v[0] = -_r*a*sin(tau);
        v[1] = _r*a*cos(tau);
        v[2] = 0;
        return _T.getRotation()*v;
	}

    Vector3D<T> ddx(double t) const
    {
        const double a = (_tend-_tstart)/_duration;
        const double tau = a*t + _tstart;

        Vector3D<double> v;
        v[0] = -_r*a*a*cos(tau);
        v[1] = -_r*a*a*sin(tau);
        v[2] = 0;
        return (_T.getRotation())*v;
    }

    double duration() const
    {
    	return _duration;
    }

    double getLength() const
    {
    	return (_tend - _tstart)*_r;
    }

	virtual ~CircularInterpolator(){}
private:
	/** @brief 开始点 */
	Vector3D<T> _p1;

	/** @brief 中间点 */
	Vector3D<T> _p2;

	/** @brief 结束点 */
	Vector3D<T> _p3;

	/** @brief 插补时长 */
	double _duration;

	/**  * @brief 映射坐标系到原坐标系的变换矩阵 */
	HTransform3D<T> _T;

	/** @brief 圆心x坐标 */
	double _cx;

	/** @brief 圆心y坐标 */
	double _cy;

	/** @brief 半径 */
	double _r;

	/** @brief 映射坐标系下的开始角度 */
	double _tstart;

	/** @brief 映射坐标系下的结束角度 */
	double _tend;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* CIRCULARINTERPOLATOR_H_ */
