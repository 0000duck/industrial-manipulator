/*
 * Sampler.h
 *
 *  Created on: Oct 13, 2017
 *      Author: a1994846931931
 */

#ifndef SAMPLER_H_
#define SAMPLER_H_

# include "Interpolator.h"
# include <functional>

using std::vector;
using robot::trajectory::Interpolator;

namespace robot {
namespace trajectory {

template< class T>
	class Sampler {
	public:
		Sampler(){}

		/**
		 * @brief 插补器采样函数
		 * @param ipr [in] 插补器
		 * @param count [in] 采样点数
		 * @param method [in] 获取方法. 对于Interpolator, 可以选择
		 * 	- "x" 默认
		 * 	- "dx"
		 * 	- "ddx"
		 *
		 * 或其他方法.
		 * @return 线性平等间距采样结果
		 */
		static vector<T> sample(std::shared_ptr<Interpolator<T> > ipr, int count, const char* method_c="x")
		{
			std::function<T(double)> method;
			if (std::string(method_c) == std::string("x"))
				method = [&](double t){return ipr->x(t);};
			if (std::string(method_c) == std::string("dx"))
				method = [&](double t){return ipr->x(t);};
			if (std::string(method_c) == std::string("ddx"))
				method = [&](double t){return ipr->x(t);};
			else
				method = [&](double t){return ipr->x(t);};
			double duration = ipr->duration();
			double dt = duration/(count - 1);
			vector<double> t;
			for (int i=0; i<count; i++)
			{
				t.push_back(i*dt);
			}
			return sample(t, method);
		}

		/**
		 * @brief 通用采样方法
		 * @param t [in] 时间列表
		 * @param method [in] 用于采样的计算函数
		 * @return 与时间列表相对应的返回值列表
		 */
		static vector<T> sample(vector<double> t, std::function<T(double)> method)
		{
			vector<T> result;
			for (auto it=t.begin(); it != t.end(); it++)
				result.push_back(method(*it));
			return result;
		}
		virtual ~Sampler(){}
	};

} /* namespace trajectory */
} /* namespace robot */

#endif /* SAMPLER_H_ */
