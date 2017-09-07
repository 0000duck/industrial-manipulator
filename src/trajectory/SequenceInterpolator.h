/**
 * @brief SequenceInterpolator类
 * @date Sep 6, 2017
 * @author a1994846931931
 */

#ifndef SEQUENCEINTERPOLATOR_H_
#define SEQUENCEINTERPOLATOR_H_

# include "Interpolator.h"
# include <vector>

using namespace robot::trajectory;

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */

/**
 * @brief 插补器序列
 *
 * 将一系列插补器拼接成一个新的插补器
 */
template <class T>
class SequenceInterpolator: public Interpolator<T> {
public:
	/** @brief 默认构造函数 */
	SequenceInterpolator();
	/**
	 * @brief 增添插补器
	 * @param interpolator [in] 增添的插补器
	 *
	 * 添加插补器到末端
	 */
	void addInterpolator(Interpolator<T>* interpolator)
	{
		_interpolatorSequence.push_back(interpolator);
		if (_timeSequence.size() == 0)
			_timeSequence.push_back(interpolator->duration());
		else
			_timeSequence.push_back(interpolator->duration() + *(_timeSequence.end()));
	}

	T x(double t) const
	{
		int i=0;
		for (; i<_interpolatorSequence.size(); i++)
		{
			if (_timeSequence[i] > t)
				break;
		}
		double interpolatorT = t - ((i == 0)? 0:_timeSequence[i - 1]);
		return _interpolatorSequence[i]->x(interpolatorT);
	}

	T dx(double t) const
	{
		int i=0;
		for (; i<_interpolatorSequence.size(); i++)
		{
			if (_timeSequence[i] > t)
				break;
		}
		double interpolatorT = t - ((i == 0)? 0:_timeSequence[i - 1]);
		return _interpolatorSequence[i]->dx(interpolatorT);
	}

	T ddx(double t) const
	{
		int i=0;
		for (; i<_interpolatorSequence.size(); i++)
		{
			if (_timeSequence[i] > t)
				break;
		}
		double interpolatorT = t - ((i == 0)? 0:_timeSequence[i - 1]);
		return _interpolatorSequence[i]->ddx(interpolatorT);
	}

	double duration() const
	{
		return *(_timeSequence.end());
	}

	virtual ~SequenceInterpolator();
private:
	/** @brief 插补器序列 */
	std::vector<Interpolator<T>*> _interpolatorSequence;

	/** @brief 插补器最大时间序列 */
	std::vector<double> _timeSequence;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* SEQUENCEINTERPOLATOR_H_ */
