/**
 * @brief ConvertedInterpolator类
 * @date: Sep 7, 2017
 * @author a1994846931931
 */

#ifndef CONVERTEDINTERPOLATOR_H_
#define CONVERTEDINTERPOLATOR_H_

# include "Interpolator.h"
# include <vector>
# include "../math/Q.h"
# include "../common/printAdvance.h"
# include <memory>

namespace robot {
namespace trajectory {

/**
 * @addtogroup trajectory
 * @{
 */

/**
 * @brief 转换类型的插补器模板类
 *
 * 可以将类型B构造成输出类型为T的插补器, 类型B默认是一种插补器. 在默认情况下,
 * 尝试将B插补器的输出强制类型转换成T类的输出. 类型B也可以是别的种类, 例如一个元素为
 * 插补器指针的vector, 甚至是一个常量.
 */
template <class B, class T>
class ConvertedInterpolator: public Interpolator<T> {
public:
	typedef std::shared_ptr<ConvertedInterpolator<B, T> > ptr;
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

	T x(double t) const
	{
		return T(_OriginalInterpolator->x(t));
	}

	T dx(double t) const
	{
		return T(_OriginalInterpolator->dx(t));
	}

	T ddx(double t) const
	{
		return T(_OriginalInterpolator->ddx(t));
	}

	double duration() const
	{
		return _OriginalInterpolator->duration();
	}
	virtual ~ConvertedInterpolator(){}
private:
	/** @brief 源插补器 */
	Interpolator<B>* _OriginalInterpolator;
};

/**
 * @brief 转换自double类型插补器指针容器, 转换至Q
 */
template<>
class ConvertedInterpolator<std::vector<Interpolator<double>* > , robot::math::Q>: public Interpolator<robot::math::Q> {
public:
	ConvertedInterpolator(std::vector<Interpolator<double>* > origin)
	{
		_interpolatorList.assign(origin.begin(), origin.end());
		_size = _interpolatorList.size();
		if (_size <= 0)
			throw ("错误: 用于构造Q类型的插补器容器大小不能为0!");
	}

	robot::math::Q x(double t) const
	{
		robot::math::Q q = robot::math::Q::zero(_size);
		for (int i=0; i<_size; i++)
			q(i) = _interpolatorList[i]->x(t);
		return q;
	}

	robot::math::Q dx(double t) const
	{
		robot::math::Q q = robot::math::Q::zero(_size);
		for (int i=0; i<_size; i++)
			q(i) = _interpolatorList[i]->dx(t);
		return q;
	}

	robot::math::Q ddx(double t) const
	{
		robot::math::Q q = robot::math::Q::zero(_size);
		for (int i=0; i<_size; i++)
			q(i) = _interpolatorList[i]->ddx(t);
		return q;
	}

	double duration() const
	{
		return _interpolatorList[0]->duration();
	}
	virtual ~ConvertedInterpolator(){}
private:
	/** @brief 源插补器列表 */
	std::vector<Interpolator<double>* > _interpolatorList;

	/** @brief 插补器列表大小 */
	int _size;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* CONVERTEDINTERPOLATOR_H_ */
