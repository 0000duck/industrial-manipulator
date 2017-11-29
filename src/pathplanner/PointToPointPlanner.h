/**
 * @brief PointToPointPlanner类
 * @date Sep 5, 2017
 * @author a1994846931931
 */

#ifndef POINTTOPOINTPLANNER_H_
#define POINTTOPOINTPLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"
# include "../pathplanner/SmoothMotionPlanner.h"
# include "../trajectory/ConvertedInterpolator.h"

using robot::math::Q;
using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/** @addtogroup pathplanner
 * @{
 */

/**
 * @brief 点到点平滑路径规划器
 *
 * 用于从一个Q到另一个Q的平滑插补, 初始和结束的速度, 加速度都为0.
 * @warning 该规划各个关节都单独做S型速度曲线规划, 运行速度的改变可能导致
 * 规划路径有所变化. 若需要不同速度下路径不变的MoveJ规划, 可使用QtoQPlanner.
 */
class PointToPointPlanner {
public:
	/**
	 * @brief 构造函数
	 * @param h [in] 加加速度
	 * @param aMax [in] 最大加速度
	 * @param vMax [in] 最大速度
	 */
	PointToPointPlanner(Q h, Q aMax, Q vMax);

	/**
	 * @brief 规划路径
	 * @param qStart [in] 开始位置
	 * @param qEnd [in] 结束位置
	 * @return
	 */
	Interpolator<Q>::ptr query(Q qStart, Q qEnd);

	virtual ~PointToPointPlanner();
private:
	/** @brief Q的大小 */
	int _size;

	/** @brief 记录的加加速度 */
	Q _h;

	/** @brief 记录的最大加速度 */
	Q _aMax;

	/** @brief 记录的最大速度 */
	Q _vMax;
};

/** @} */
} /* namespace pathplanner */
} /* namespace robot */

#endif /* POINTTOPOINTPLANNER_H_ */
