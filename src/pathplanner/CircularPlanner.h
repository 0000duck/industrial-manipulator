/*
 * CircularPlanner.h
 *
 *  Created on: Sep 18, 2017
 *      Author: a1994846931931
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

class CircularPlanner {
public:
	CircularPlanner(Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine, double vMaxAngle, double aMaxAngle, double hAngle,
			std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink::ptr serialLink);
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

} /* namespace pathplanner */
} /* namespace robot */

#endif /* CIRCULARPLANNER_H_ */
