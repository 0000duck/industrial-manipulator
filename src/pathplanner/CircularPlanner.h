/**
 * @brief CircularPlanner
 * @date Sep 18, 2017
 * @author a1994846931931
 */

#ifndef CIRCULARPLANNER_H_
#define CIRCULARPLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"
# include "../ik/IKSolver.h"
# include "SmoothMotionPlanner.h"
# include <vector>
# include "../model/SerialLink.h"
# include "../trajectory/CompositeInterpolator.h"
# include "../trajectory/LinearInterpolator.h"
# include "../trajectory/CircularTrajectory.h"
# include <memory>

using robot::math::Q;
using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * 圆弧路径规划器
 */
class CircularPlanner {
public:
	/**
	 * @brief 构造函数
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
	CircularPlanner(Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine, double vMaxAngle, double aMaxAngle, double hAngle,
			std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink::ptr serialLink);

	/**
	 * @brief 询问路径
	 * @param qStart [in] 圆弧开始点的关节数值
	 * @param qIntermediate [in] 圆弧中间点的关节数值
	 * @param qEnd [in] 圆弧结束点的关节数值
	 * @param speedRatio [in] 速度占最大速度的比例
	 * @param accRatio [in] 加速度占最大加速度的比例
	 * @return
	 */
	CircularTrajectory::ptr query(const Q qStart, const Q qIntermediate, const Q qEnd, double speedRatio, double accRatio) const;
	virtual ~CircularPlanner(){}
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
    robot::model::SerialLink::ptr _serialLink;

    /** @brief 平滑路径规划器 */
    SmoothMotionPlanner _smPlanner;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* CIRCULARPLANNER_H_ */
