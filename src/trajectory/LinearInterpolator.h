/**
 * @brief LinearInterpolater类
 * @date Sep 1, 2017
 * @author a1994846931931
 */

#ifndef LINEARINTERPOLATOR_H_
#define LINEARINTERPOLATOR_H_

# include <vector>
# include "Interpolator.h"
# include "../math/Quaternion.h"
# include "../math/Rotation3D.h"
# include "../math/HTransform3D.h"
# include "../math/Quaternion.h"
# include "../math/Q.h"

using namespace robot::math;

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
	/**
	 * @brief 构造线性插补器
	 * @param start [in] 开始位置
	 * @param end [in] 终点位置
	 * @param duration [in] 插补时长
	 */
	LinearInterpolator(const T& start, const T& end, double duration):
		_a(start),
		_b((end - start)/duration),
		_vel(_b),
		_acc(start*0),
		_duration(duration)
	{
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
		_vel = (Quaternion((1.0*_theta/_duration), _n)).toRotation3D();
		_acc = Rotation3D<T>::identity();
	}

	virtual ~LinearInterpolator(){}

	Rotation3D<T> x(double t) const
	{
		return (Quaternion((t*_theta/_duration), _n)).toRotation3D();
	}

	Rotation3D<T> dx(double t) const
	{
		return _vel; // TODO 定义无意义
	}

	Rotation3D<T> ddx(double t) const
	{
		return _acc; // TODO 定义无意义
	}

	double duration() const
	{
		return _duration;
	}
private:
	Rotation3D<T> _start;
	Rotation3D<T> _end;
	Quaternion _quartStart;
	Quaternion _quartEnd;
	double _duration;
	double _theta;
	Vector3D<T> _n;
	Rotation3D<T> _vel;
	Rotation3D<T> _acc;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* LINEARINTERPOLATOR_H_ */
