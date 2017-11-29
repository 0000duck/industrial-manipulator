/**
 * @brief TimeOptimalPlanner.h
 * @date Nov 7, 2017
 * @author a1994846931931
 */

#ifndef TIMEOPTIMALPLANNER_H_
#define TIMEOPTIMALPLANNER_H_

# include "../trajectory/SequenceInterpolator.h"
# include "../trajectory/Trajectory.h"
# include <functional>

using robot::trajectory::SequenceInterpolator;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief 时间最优规划类
 *
 * 根据沿轨迹的速度限制来进行可行的时间最优规划.
 */
class TimeOptimalPlanner {
public:
	TimeOptimalPlanner();
	virtual ~TimeOptimalPlanner();
public:
	static void optimizeVelocityRestriction(vector<double>& maxSpeed, double a, double h, double ds);

	/**
	 * @brief 根据最大速度函数时间最优s型规划
	 * @param Vm [in] 获取最大速度限制的函数
	 * @param s [in] 规划的距离
	 * @param ve [in] 期望速度
	 * @param a [in] 使用加速度
	 * @param h [in] 使用加加速度
	 * @param ds [in] 沿轨迹长度采样精度, 会影响函数内的时间采样精度. 若获取最大速度限制的函数由
	 * 采样点生成, 则设置为这个采样精度, 可以避免执行速度检查时跳过某一采样速度区间.
	 * @return 时间最优的长度-时间插补器, 速度由最大速度限制函数Vm指定
	 */
	static SequenceInterpolator<double>::ptr getOptimalLt(std::function<double(double)>& Vm, double s, double ve, double a, double h, double ds);

	/**
	 * @brief 根据已优化的最大速度采样时间最优s型规划
	 * @param optimizedMaxSpeed [in] 采样的最大速度,采样长度间隔为ds,且两个采样点之间的速度必然可以由
	 * 一个二速度规划到达. 通常要由optimizeVelocityRestriction函数进行优化.
	 * @param s [in] 规划的距离
	 * @param ve [in] 期望速度
	 * @param a [in] 使用加速度
	 * @param h [in] 使用加加速度
	 * @param ds [in] 沿轨迹长度采样精度
	 * @return 时间最优的长度-时间插补器
	 */
	static SequenceInterpolator<double>::ptr getOptimalLt(vector<double> optimizedMaxSpeed, double s, double ve, double a, double h, double ds);
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* TIMEOPTIMALPLANNER_H_ */
