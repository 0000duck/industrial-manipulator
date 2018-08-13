/**
 * @brief SequenceInterpolator类
 * @date Sep 6, 2017
 * @author a1994846931931
 */

#ifndef SEQUENCEINTERPOLATOR_H_
#define SEQUENCEINTERPOLATOR_H_

# include "Interpolator.h"
# include "ConvertedInterpolator.h"
# include <vector>
# include "../common/printAdvance.h"
# include <functional>
# include <memory>

using namespace robot::trajectory;
using namespace robot::common;

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
	typedef std::shared_ptr<SequenceInterpolator<T> > ptr;
	/** @brief 默认构造函数 */
	SequenceInterpolator(){}
	/**
	 * @brief 增添插补器
	 * @param interpolator [in] 增添的插补器
	 *
	 * 添加插补器到末端
	 */
	void addInterpolator(std::shared_ptr<Interpolator<T> > interpolator)
	{
		_interpolatorSequence.push_back(interpolator);
		if (_timeSequence.empty())
		{
			_timeSequence.push_back(interpolator->duration());
		}
		else
		{
			_timeSequence.push_back(interpolator->duration() + *(_timeSequence.end() - 1));
		}
	}

	/**
	 * @brief 拼接插补器
	 * @param interpolator [in] 拼接的插补器
	 *
	 * 用于拼接插补器. 下一个插补器要加上上一个插补器的结束值.
	 */
	void appendInterpolator(std::shared_ptr<Interpolator<double> > interpolator)
	{
		if (_timeSequence.size() == 0)
		{
			_interpolatorSequence.push_back(interpolator);
		}
		else
		{
			int index = (int)_interpolatorSequence.size() - 1;
			double duration = _interpolatorSequence[index]->duration();
			double end = _interpolatorSequence[index]->x(duration);
			ConvertedInterpolator<double, double>::ptr movedIpr(new ConvertedInterpolator<double, double>(
					interpolator, [=](double x){return x + end;}, [](double x){return x;}, [](double x){return x;}));
			_interpolatorSequence.push_back(movedIpr);
		}
		if (_timeSequence.size() == 0)
			_timeSequence.push_back(interpolator->duration());
		else
		{
			_timeSequence.push_back(interpolator->duration() + *(_timeSequence.end() - 1));
		}

	}

	T x(double t) const
	{
		if (_timeSequence.empty())
			throw("错误<SequenceInterpolator>: 插补器时间序列为空!");
		auto it = upper_bound(_timeSequence.begin(), _timeSequence.end(), t);
		if (it == _timeSequence.end()) it--;
		int idx = it - _timeSequence.begin();
		return (it == _timeSequence.begin())? _interpolatorSequence[0]->x(t):
				_interpolatorSequence[idx]->x(t - _timeSequence[idx - 1]);
	}

	T dx(double t) const
	{
		if (_timeSequence.empty())
			throw("错误<SequenceInterpolator>: 插补器时间序列为空!");
		auto it = upper_bound(_timeSequence.begin(), _timeSequence.end(), t);
		if (it == _timeSequence.end()) it--;
		int idx = it - _timeSequence.begin();
		return (it == _timeSequence.begin())? _interpolatorSequence[0]->dx(t):
				_interpolatorSequence[idx]->dx(t - _timeSequence[idx - 1]);
	}

	T ddx(double t) const
	{
		if (_timeSequence.empty())
			throw("错误<SequenceInterpolator>: 插补器时间序列为空!");
		auto it = upper_bound(_timeSequence.begin(), _timeSequence.end(), t);
		if (it == _timeSequence.end()) it--;
		int idx = it - _timeSequence.begin();
		return (it == _timeSequence.begin())? _interpolatorSequence[0]->ddx(t):
				_interpolatorSequence[idx]->ddx(t - _timeSequence[idx - 1]);
	}

	double duration() const
	{
		return *(_timeSequence.end() - 1);
	}

	virtual ~SequenceInterpolator(){}
private:
	/** @brief 插补器序列 */
	std::vector<std::shared_ptr<Interpolator<T> > > _interpolatorSequence;

	/** @brief 插补器最大时间序列 */
	std::vector<double> _timeSequence;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* SEQUENCEINTERPOLATOR_H_ */
