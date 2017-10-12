/*
 * MultiLineArcBlendPlanner.h
 *
 *  Created on: Sep 26, 2017
 *      Author: a1994846931931
 */

#ifndef MULTILINEARCBLENDPLANNER_H_
#define MULTILINEARCBLENDPLANNER_H_

# include "../trajectory/MLABTrajectory.h"
# include "../math/Q.h"
# include "../model/SerialLink.h"
# include "../math/HTransform3D.h"
# include "../trajectory/CircularInterpolator.h"
# include "../trajectory/LinearInterpolator.h"

using namespace robot::trajectory;
using std::vector;
using robot::model::SerialLink;
using robot::math::HTransform3D;

namespace robot {
namespace pathplanner {

/**
 * @brief 连续规划-多直线圆弧混合路径规划器
 */
class MultiLineArcBlendPlanner {
public:
	MultiLineArcBlendPlanner(Q dqLim, Q ddqLim,
			std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink::ptr serialLink);
	MLABTrajectory::ptr query(const vector<Q>& path, const vector<double>& arcRatio, vector<double>& velocity, vector<double>& acceleration, vector<double>& jerk);
	vector<SequenceInterpolator<double>::ptr> getLt(double arcSize, double lineSize, vector<Trajectory::ptr>& trajectoryIpr, vector<double>& velocity, vector<double>& acceleration, vector<double>& jerk);
	virtual ~MultiLineArcBlendPlanner();
private:
private:
	/** @brief 关节个数 */
	int _size;

	/** @brief 逆解器 */
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

	/** @brief 关节下限 */
	const Q _qMin;

	/** @brief 关节上限 */
	const Q _qMax;

	/** @brief 关节最大速度 */
	const Q _dqLim;

	/** @brief 关节最大加速度 */
	const Q _ddqLim;

    /** @brief 机器人的模型 */
	SerialLink::ptr _serialLink;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* MULTILINEARCBLENDPLANNER_H_ */
