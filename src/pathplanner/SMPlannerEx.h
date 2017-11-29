/**
 * @brief SMPlannerEx
 * @date Sep 25, 2017
 * @author a1994846931931
 */

#ifndef SMPLANNEREX_H_
#define SMPLANNEREX_H_

# include "../trajectory/Interpolator.h"
# include <vector>
# include "../trajectory/SequenceInterpolator.h"

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief 加强S速度曲线规划器
 *
 * 使用时主要关注四个函数:
 *  - query()
 *  - query_flexible() (两种)
 *  - query_stop()
 */
class SMPlannerEx {
public:
	SMPlannerEx();

	/**
	 * @brief 在v1到v2速度之间进行规划
	 * @param start [in] 初始位置
	 * @param s [in] 要到达的距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 末端速度
	 * @param stop [in] 最后是否要停下来
	 * @return 插补器指针
	 *
	 * 速度大于0. 当距离不够会报错. 若stop=false, 则先加速或减速到v2然后匀速跑完线段.
	 * 若stop=true, 则先加速或减速到v2, 匀速一段距离, 然后再降速到0并刚好到达指定距离.
	 * 规划的路径始末的加速度都为0.
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr query(double start, double s, double h, double aMax, double v1, double v2, bool stop=false) const;

	/**
	 * @brief 柔性速度规划(enhanced query)
	 * @param start [in] 初始位置
	 * @param s [in] 要到达的距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 期望到达的末端速度
	 * @param realV2 [out] 实际到达的末端速度
	 * @param stop [in] 是否最后要停下来
	 * @return 插补器指针
	 *
	 * 规划器尝试v1到v2的规划, 如果在有限距离内无法达到速度v2, 不报错而是做尽量到达v2速度
	 * 的规划, 实际达到的速度realV2通过参数返回.
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr query_flexible(double start, double s, double h, double aMax, double v1, double v2, double &realV2, bool stop=false) const;

	/**
	 * @brief 柔性三速度规划
	 * @param start [in] 初始位置
	 * @param s [in] 要到达的距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 期望到达的中段速度
	 * @param v3 [in] 期望到达的末端速度
	 * @param realV2 [out] 实际到达的中段速度
	 * @param realV3 [out] 实际到达的末端速度
	 * @return 插补器指针
	 *
	 * 规划器尝试规划从v1到v2, 匀速, 最后从v2到v3的速度规划, 如果有限距离不够, 则用柔性规划做v1到v3的规划(
	 * 无视v2条件), 此时返回的realV2 = realV3
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr query_flexible(
			double start,
			double s,
			double h,
			double aMax,
			double v1,
			double v2,
			double v3,
			double &realV2,
			double &realV3) const;

	/**
	 * @brief 暂停规划 - 从当前速度加速度情况下直接减速到零(唯一可以指定初始加速度的规划)
	 * @param s0 [in] 初始位置
	 * @param v0 [in] 初始速度
	 * @param a0 [in] 初始加速度
	 * @param h [in] 加加速度限制
	 * @param aMax [in] 加速度限制
	 * @return 插补器指针
	 * @note 不可指定距离, 保证加速度平滑的情况下最快速度停止. 可以指定初始加速度.
	 * 需要指定距离时调用query或query_flexible并将stop参数设置为true
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr query_stop(double s0, double v0, double a0, double h, double aMax);

	/**
	 * @brief 判断要满足v1, v2, h, aMax条件下距离s是否足够
	 * @param s [in] 要到达的距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 期望末端速度
	 * @param stop [in] 最后是否要停下来
	 * @param realV2 [out] 实际能到达的末端速度
	 * @return 是否能在满足期望末端速度的情况下到达距离s
	 */
	bool checkDitance(double s, double h, double aMax, double v1, double v2, double &realV2, bool stop=false) const;

	/**
	 * @brief 从速度v1达到v2所需要的最短距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 期望末端速度
	 * @return 从速度v1达到v2所需要的最短距离
	 */
	double queryMinDistance(double h, double aMax, double v1, double v2) const;

	/**
	 * 二分法求末端可以达到最大的速度
	 * @param s [in] 要到达的距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 期望末端速度
	 * @return 距离限制下可达到的最大速度
	 */
	double queryMaxSpeed(double s, double h, double aMax, double v1, double v2) const;

	virtual ~SMPlannerEx(){}
private:
	/** @brief query() stop=true的实现 */
	robot::trajectory::SequenceInterpolator<double>::ptr query_stop(double start, double s, double h, double aMax, double v1, double v2) const;

	/** @brief query_flexible() stop=true的实现 */
	robot::trajectory::SequenceInterpolator<double>::ptr query_flexible_stop(double start, double s, double h, double aMax, double v1, double v2, double &realV2) const;

	/**
	 * @brief 停止段判断要满足v1, v2, h, aMax条件下距离s是否足够
	 * @param s [in] 要到达的距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 期望中途匀速段速度
	 * @param realV2 [out] 实际能到达的末端速度. 若结果为v1, 不表明一定能在距离范围内停止, 应当用
	 * queryMinDistance再次排查
	 * @return 是否能在满足期望末端速度的情况下到达距离s
	 */
	bool checkDitance_stop(double s, double h, double aMax, double v1, double v2, double &realV2) const;

	bool checkDitance(double s, double h, double aMax, double v1, double v2, double v3, double &realV2, double &realV3) const;

	/**
	 * @brief 停止段先从速度v1达到v2再降到速度0所需要的最短距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 期望中途匀速段速度
	 * @return 停止段先从速度v1达到v2再降到速度0所需要的最短距离
	 */
	double queryMinDistance_stop(double h, double aMax, double v1, double v2) const;
	/**
	 * @brief 二分法求结束段中途可以达到最大的速度
	 * @param s [in] 要到达的距离
	 * @param h [in] 限制的最大加加速度
	 * @param aMax [in] 限制的最大加速度
	 * @param v1 [in] 初始速度
	 * @param v2 [in] 期望中途匀速段速度
	 * @return 可达到的速度v2. 若结果为v1, 不表明一定能在距离范围内停止, 应当用
	 * queryMinDistance再次排查
	 */
	double queryMaxSpeed_stop(double s, double h, double aMax, double v1, double v2) const;
	robot::trajectory::SequenceInterpolator<double>::ptr threeLineMotion(double start, double s, double h, double aMax, double v1, double v2) const;
	robot::trajectory::SequenceInterpolator<double>::ptr fourLineMotion(double start, double s, double h, double aMax, double v1, double v2) const;
	robot::trajectory::SequenceInterpolator<double>::ptr threeLineMotion(double start, double s, double h, double aMax, double v1) const;
	robot::trajectory::SequenceInterpolator<double>::ptr fourLineMotion(double start, double s, double h, double aMax, double v1) const;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* SMPLANNEREX_H_ */
