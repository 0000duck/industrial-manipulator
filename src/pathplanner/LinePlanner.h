/**
 * @brief LinePlanner类
 * @date Sep 11, 2017
 * @author a1994846931931
 */

#ifndef LINEPLANNER_H_
#define LINEPLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"
# include "../ik/IKSolver.h"
# include "SmoothMotionPlanner.h"
# include <vector>
# include "../model/SerialLink.h"
# include "../trajectory/LineInterpolator.h"
# include "../trajectory/LinearInterpolator.h"
# include <memory>

using robot::math::Q;
using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/** @addtogroup pathplanner
 * @{
 */

/**
 * @brief 直线规划器
 *
 * 给定两个点, 规划中间的路径, 返回Q插补器. 需要指明机器人各个关节的
 * 最大速度和最大加速度, 以及要规划路线的最大速度, 最大加速度, 以及加加速度.
 * - 如果按照路线设置的最大速度, 最大加速度, 以及加加速度规划出符合关节约束的路径, 则返回这条路径.
 * - 如果直线段有路径无法到达, 则抛出错误.
 * - 如果路径超出了关节的最大速度约束或者最大加速度约束, 则降低速度, 返回降速后的路径(插补器).
 * - 规划器会检查起始和结束位置的config参数, 若config参数不相同, 则抛出错误(直线规划器的首末config必须相同)
 */
class LinePlanner {
public:
	/**
	 * @brief 构造函数
	 * @param qMin [in] 关节下限
	 * @param qMax [in] 关节上限
	 * @param dqLim [in] 关节最大速度
	 * @param ddqLim [in] 关节最大加速度
	 * @param vMaxLine [in] 直线路径最大速度
	 * @param aMaxLine [in] 直线路径最大加速度
	 * @param hLine [in] 直线路径加加速度
	 * @param vMaxAngle [in] 直线路径最大角速度
	 * @param aMaxAngle [in] 直线路径最大角加速度
	 * @param hAngle [in] 直线路径角加加速度
	 * @param ikSolver [in] 逆解器
	 * @param serialLink [in] 机器人模型
	 */
	LinePlanner(Q qMin, Q qMax, Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine, double vMaxAngle, double aMaxAngle, double hAngle,
			std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink* serialLink);

	/**
	 * @brief 询问路径
	 * @param qStart [in] 起始位置
	 * @param qEnd [in] 终点位置
	 * @return 直线路径的Q插补器
	 */
	LineInterpolator::ptr query(const Q qStart, const Q qEnd) const;
	virtual ~LinePlanner();
private:
	/** @brief 末端预定最大直线速度 */
	double _vMaxLine;

	/** @brief 末端预定最大直线加速度 */
	double _aMaxLine;

	/** @brief 末端预定最大直线加加速度 */
	double _hLine;

	/** @brief 末端预定最大角速度 */
	double _vMaxAngle;

	/** @brief 末端预定最大角加速度 */
	double _aMaxAngle;

	/** @brief 末端预定最大角加加速度 */
	double _hAngle;

	/** @brief 逆解器 */
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

	/** @brief 关节下限 */
	Q _qMin;

	/** @brief 关节上限 */
	Q _qMax;

	/** @brief 关节最大速度 */
	Q _dqLim;

	/** @brief 关节最大加速度 */
	Q _ddqLim;

	/** @brief 关节个数 */
	int _size;

    /** @brief 机器人的模型 */
    robot::model::SerialLink* _serialLink;

    /** @brief 平滑路径规划器 */
    SmoothMotionPlanner _smPlanner;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* LINEPLANNER_H_ */
