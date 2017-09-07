/*
 * ConvertedInterpolator.h
 *
 *  Created on: Sep 7, 2017
 *      Author: a1994846931931
 */

#ifndef CONVERTEDINTERPOLATOR_H_
#define CONVERTEDINTERPOLATOR_H_

#include "Interpolator.h"

namespace robot {
namespace trajectory {

/**
 * @addtogroup trajectory
 * @{
 */

/**
 * @brief 转换类型的插补器模板类
 *
 * 可以将一个特定输出类型的插补器构造成另一个输出类型的插补器
 */
template <class T, class B>
class ConvertedInterpolator: public Interpolator<T> {
public:
	/**
	 * @brief 构造函数
	 *
	 * 记录源插补器的指针
	 * @param origin [in] 源插补器
	 */
	ConvertedInterpolator(Interpolator<B>* origin)
	{
		_OriginalInterpolator = origin;
	}

	T x(double t)
	{
		return T(_OriginalInterpolator->x(t));
	}

	T dx(double t)
	{
		return T(_OriginalInterpolator->dx(t));
	}

	T ddx(double t)
	{
		return T(_OriginalInterpolator->ddx(t));
	}
	virtual ~ConvertedInterpolator();
private:
	/** @brief 源插补器 */
	Interpolator<T>* _OriginalInterpolator;
};


/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* CONVERTEDINTERPOLATOR_H_ */
