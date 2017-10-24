/*
 * RotationPlanner.h
 *
 *  Created on: Oct 23, 2017
 *      Author: a1994846931931
 */

#ifndef ROTATIONPLANNER_H_
#define ROTATIONPLANNER_H_

# include "../trajectory/RotationInterpolator.h"
# include "../trajectory/LineTrajectory.h"
# include "Planner.h"

using robot::trajectory::RotationInterpolator;

namespace robot {
namespace pathplanner {

/**
 * @brief 纯旋转运动规划器
 */
class RotationPlanner : public Planner{
public:
	RotationPlanner(Q dqLim, Q ddqLim,
			double vMaxRot, double aMaxRot, double hRot,
			std::shared_ptr<robot::ik::IKSolver> ikSolver,
			Q start, Vector3D<double> n, double theta);

	LineTrajectory::ptr query();

	void doQuery();
	bool stop(double t, Interpolator<Q>::ptr& stopIpr);
	void resume(); //qStart为恢复点, 可以改为自动获取, 或留以作为位置误差判断
	bool isTrajectoryExist() const;
	Interpolator<Q>::ptr getQTrajectory() const;
	virtual ~RotationPlanner();
public:
	static double findReachableTheta(Q start, Vector3D<double> n, std::shared_ptr<robot::ik::IKSolver> ikSolver, double da=0.1);
private:
	/** @brief 关节最大速度 */
	Q _dqLim;

	/** @brief 关节最大加速度 */
	Q _ddqLim;

	/** @brief 末端预定最大角速度 */
	double _vMax;

	/** @brief 末端预定最大角加速度 */
	double _aMax;

	/** @brief 末端预定最大角加加速度 */
	double _h;

	/** @brief 逆解器 */
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

    /** @brief 机器人的模型 */
    robot::model::SerialLink::ptr _serialLink;

	/** @brief 关节个数 */
	int _size;

	Vector3D<double> _n;

	/**> 需要旋转的角度, 暂停后需要重置 */
	double _theta;

    /**> 记录的停止点, 用于恢复运动时检查启动点的位置正不正确 */
    Q _qStop;

    /**> 保存的config参数 */
    robot::model::Config _config;

    LineTrajectory::ptr _lineTrajectory;

	/** @brief 采样精度 rad */
	const double _da = 0.5;

	/** @brief 最少采样点数 */
	const double _countMin = 8;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* ROTATIONPLANNER_H_ */
