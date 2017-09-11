/**
 * @brief CompositeInterpolator类, LinearCompositeInterpolator类
 * @date Sep 8, 2017
 * @author a1994846931931
 */

#ifndef COMPOSITEINTERPOLATOR_H_
#define COMPOSITEINTERPOLATOR_H_

# include "Interpolator.h"
# include "../common/printAdvance.h"
# include <math.h>

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */

/**
 * @brief 复合插补器
 *
 * 用于一个主插补器@f$ f(t)@f$ 和一个double类型的映射插补器@f$ e(t) @f$ 的复合. 复合插补器的插补时长变为e(t)的插补时长.
 * - 复合后的插补器函数为:
 * 	- @f$ f(e(t)) @f$ .
 * - 复合插补器的一次导为:
 * 	- @f$ \frac {df(e(t))}{dt} = \dot f(e(t)) \cdot\dot e(t)@f$;
 * - 二次导为:
 * 	- @f$ \frac {d^2f(e(t))}{dt^2} = {f(e(t))}^{(2)} \cdot {\dot e(t)}^2 + \dot f(e(t))\cdot {e(t)}^{(2)}@f$
 */
template <class T>
class CompositeInterpolator: public Interpolator<T> {
public:
	/**
	 * @brief 构造函数
	 * @param interpolator [in] 主插补器
	 * @param mapper [in] 映射插补器
	 */
	CompositeInterpolator(Interpolator<T>* interpolator, Interpolator<double>* mapper)
	{
		_interpolator = interpolator;
		_mapper = mapper;
		if (fabs(mapper->x(mapper->duration()) - interpolator->duration()) > 1e-3)
			common::println("警告: 复合插补器, mapper的范围与源插补器的时间周期不一致(误差超出0.001)");
	}

	T x(double t) const
	{
		return _interpolator->x(_mapper->x(t));
	}

	T dx(double t) const
	{
		return (_interpolator->dx(_mapper->x(t)))*(_mapper->dx(t));
	}

	T ddx(double t) const
	{
		return (_interpolator->ddx(_mapper->x(t)))*pow((_mapper->dx(t)), 2) + (_interpolator->dx(_mapper->x(t)))*(_mapper->ddx(t));
	}

	double duration() const
	{
		return _mapper->duration();
	}
	virtual ~CompositeInterpolator(){}
private:
	/**
	 * @brief 主插补器地址
	 */
	Interpolator<T>* _interpolator;

	/**
	 * @brief 映射器地址
	 */
	Interpolator<double>* _mapper;
};

/**
 * @brief 映射器@f$ e(t) = k\cdot t @f$ 的复合插补器
 *
 * @f$ k为系数@f$ @f$ e(t) = k\cdot t @f$
 * - 复合后的插补器函数为:
 * 	- @f$ f(k\cdot t)) @f$ .
 * - 复合插补器的一次导为:
 * 	- @f$ \frac {df(k\cdot t)}{dt} = k\cdot f(k\cdot t))@f$;
 * - 二次导为:
 * 	- @f$ \frac {d^2f(k\cdot t)}{dt^2} = k^2 {f(k\cdot t)}^{(2)}@f$
 */
template <class T>
class LinearCompositeInterpolator: public Interpolator<T> {
public:
	/**
	 * @brief 构造函数
	 * @param interpolator [in] 主插补器
	 * @param factor [in] 系数
	 */
	LinearCompositeInterpolator(Interpolator<T>* interpolator, double factor)
	{
		_interpolator = interpolator;
		_factor = factor;
		_duration = interpolator->duration()/factor;
	}

	T x(double t) const
	{
		return _interpolator->x(_factor*t);
	}

	T dx(double t) const
	{
		return _factor*(_interpolator->dx(_factor*t));
	}

	T ddx(double t) const
	{
		return _factor*_factor*(_interpolator->ddx(_factor*t));
	}

	double duration() const
	{
		return _duration;
	}
	virtual ~LinearCompositeInterpolator(){}
private:
	Interpolator<T>* _interpolator;
	double _factor;
	double _duration;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* COMPOSITEINTERPOLATOR_H_ */
