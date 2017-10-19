/**
 * @brief LinearInterpolater类
 * @date Sep 1, 2017
 * @author a1994846931931
 */

#ifndef LINEARINTERPOLATOR_H_
#define LINEARINTERPOLATOR_H_

# include <vector>
# include "../common/printAdvance.h"
# include "Interpolator.h"
# include "../math/Quaternion.h"
# include "../math/Rotation3D.h"
# include "../math/HTransform3D.h"
# include "../math/Quaternion.h"
# include "../math/Q.h"
# include <memory>
# include "../ik/IKSolver.h"

using namespace robot::math;
using namespace robot::common;

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */

/**
 * @brief 线性插值器模板类
 *
 * 用于常量（例如double, Q...)的插值. <br>
 * 给定起始点 \f$\mathbf{s}\f$, 终点位置 \f$\mathbf{e}\f$ 以及插补时长 \f$ d\f$
 * 线性插补器的值由 \f$\mathbf{x}(t)=\mathbf{s} +
 * (\mathbf{e}-\mathbf{s})*t/d\f$给出. <br>
 *
 * 旋转矩阵的差值用到四元数的概念, 具体实现参照LinearInterpolator<Rotation3D<T> >的内容
 */
template <class T>
class LinearInterpolator: public Interpolator<T> {
public:
	using ptr = std::shared_ptr<LinearInterpolator>;
	/**
	 * @brief 构造线性插补器
	 * @param start [in] 开始位置
	 * @param end [in] 终点位置
	 * @param duration [in] 插补时长
	 */
	LinearInterpolator(const T& start, const T& end, double duration):
		_a(start),
		_acc(start*0),
		_duration(duration)
	{
		if (duration == 0)
		{
			_b = start*0;
			_vel = _b;
		}
		else
		{
			_b = (end - start)/duration;
			_vel = _b;
		}
	}
	virtual ~LinearInterpolator(){}

	T x(double t) const
	{
		return _a + _b*t;
	}

	T dx(double t) const
	{
		return _vel;
	}

	T ddx(double t) const
	{
		return _acc;
	}

	double duration() const
	{
		return _duration;
	}
private:
	T _a;
	T _b;
	T _vel;
	T _acc;
	double _duration;
};

template <class T>
class LinearInterpolator<Rotation3D<T> >: public Interpolator<Rotation3D<T> >{
public:
	typedef typename std::shared_ptr<LinearInterpolator> ptr;
	/**
	 * @brief 构造函数
	 * @param start [in] 开始位置
	 * @param end [in] 结束位置
	 * @param duration [in] 插补时长
	 */
	LinearInterpolator(const Rotation3D<T>& start,
			const Rotation3D<T>& end,
			double duration):
				_start(start),
				_end(end),
				_quartStart(start),
				_quartEnd(end),
				_duration(duration)
	{
		Quaternion deltaQuart = _quartStart.conjugate()*_quartEnd;
		if (deltaQuart.r() < 0)
			deltaQuart = -deltaQuart;
		Quaternion::rotVar var = deltaQuart.getRotationVariables();
		_theta = var.theta;
		_n = var.n;
		_vel = (duration == 0)? 0 : (_theta/duration);
		_acc = _vel*_vel;
		_quartStart.print();
	}

	virtual ~LinearInterpolator(){}

	Rotation3D<T> x(double t) const
	{
		return (_quartStart*Quaternion((t*_vel), _n)).toRotation3D();
	}

	Rotation3D<T> dx(double t) const
	{
		double theta = t*_theta/_duration;
		double s = sin(theta);
		double c = cos(theta);
		double n1 = _n(0);
		double n2 = _n(1);
		double n3 = _n(2);
		double n12 = n1*n1;
		double n22 = n2*n2;
		double n32 = n3*n3;
		double n1n2 = n1*n2;
		double n1n3 = n1*n3;
		double n2n3 = n2*n3;
		return (Rotation3D<T>(
				-s*(n22 + n32), s*n1n2 + c*n3, s*n1n3 - c*n2,
				s*n1n2 - c*n3, -s*(n12 + n32), s*n2n3 + c*n1,
				s*n1n3 + c*n2, s*n2n3 - c*n1, -s*(n12 + n22)))*_vel;
	}

	Rotation3D<T> ddx(double t) const
	{
		double theta = t*_theta/_duration;
		double s = sin(theta);
		double c = cos(theta);
		double n1 = _n(0);
		double n2 = _n(1);
		double n3 = _n(2);
		double n12 = n1*n1;
		double n22 = n2*n2;
		double n32 = n3*n3;
		double n1n2 = n1*n2;
		double n1n3 = n1*n3;
		double n2n3 = n2*n3;
		return (Rotation3D<T>(
				-c*(n22 + n32), c*n1n2 - s*n3, c*n1n3 + s*n2,
				c*n1n2 + s*n3, -c*(n12 + n32), c*n2n3 - s*n1,
				c*n1n3 - s*n2, c*n2n3 + s*n1, -c*(n12 + n22)))*_acc;
	}

	double duration() const
	{
		return _duration;
	}
private:
	const Rotation3D<T> _start;
	const Rotation3D<T> _end;
	const Quaternion _quartStart;
	const Quaternion _quartEnd;
	const double _duration;
	double _theta;
	Vector3D<T> _n;
	double _vel;
	double _acc;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* LINEARINTERPOLATOR_H_ */
