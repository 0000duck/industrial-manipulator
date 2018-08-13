/**
 * @brief QBlend类
 * @date Sep 13, 2017
 * @author a1994846931931
 */

#ifndef QBLEND_H_
#define QBLEND_H_

# include "../math/Q.h"
# include "../trajectory/Interpolator.h"
# include "../kinematics/State.h"

using robot::math::Q;
using std::vector;
using robot::kinematic::State;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief Joint模式的混合
 *
 * 对每个关节进行五次多项式插补, 使得每个关节都能满足开始和结束位置的三个约束(位置
 * , 速度和加速度). 需要预先指定一个插补的时长, 一般可由被混合路径部分的总时长给
 * 出, 然后混合器按照这个时长进行规划. 然后检查关节的速度及加速度约束, 求出k:  <br>
 * @f$ k = max\left\{
 * max \left| \frac{v_i}{v_{max}} \right|,
 * max \sqrt{\left| \frac{a_i}{a_{max}} \right|}
 * \right\} @f$ <br>
 * 如果@f$ k_{min} < k < 1.0@f$ 则返回, 否则修改时长t进行迭代求解.
 */
class QBlend {
public:
	/**
	 * @brief 构造函数
	 * @param aMax [in] 最大加速度
	 * @param vMax [in] 最大速度
	 * @param qMin [in] 关节下限
	 * @param qMax [in] 关节上限
	 */
	QBlend(Q aMax, Q vMax, Q qMin, Q qMax);

	/**
	 * @brief 规划
	 * @param qStart [in] 开始状态
	 * @param qEnd [in] 结束状态
	 * @param maxDuration [in] 最大时长(估计时长)
	 * @return
	 */
	Interpolator<Q>::ptr query(State qStart, State qEnd, double maxDuration);

	virtual ~QBlend();
private:
	/** @brief Q的大小 */
	int _size;

	/** @brief 记录的最大加速度 */
	Q _aMax;

	/** @brief 记录的最大速度 */
	Q _vMax;

	/** @brief 关节下限 */
	Q _qMin;

	/** @brief 关节上限 */
	Q _qMax;

	/** @brief 至少要达到的速度(加速度)百分比 */
	const double _kMin = 0.9;
};

/**@}*/

} /* namespace pathplanner */
} /* namespace robot */

#endif /* QBLEND_H_ */
