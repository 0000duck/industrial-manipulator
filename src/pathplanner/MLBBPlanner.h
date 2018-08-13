/*
 * MLBBPlanner.h
 *
 *  Created on: Nov 23, 2017
 *      Author: a1994846931931
 */

#ifndef MLBBPLANNER_H_
#define MLBBPLANNER_H_

# include "../ik/IKSolver.h"
# include "MultiLineArcBlendPlanner.h"
# include <vector>

using robot::ik::IKSolver;
using robot::model::SerialLink;
using std::vector;

namespace robot {
namespace pathplanner {

/**
 * @brief 贝塞尔曲线混合的连续路径规划器
 */
class MLBBPlanner {
public:
	MLBBPlanner(
			Q& dqLim,
			Q& ddqLim,
			IKSolver::ptr solver,
			vector<Q>& path,
			vector<double>& ratio,
			vector<double>& velocity,
			vector<double>& acceleration,
			vector<double>& jerk);

	void query();

	vector<SequenceInterpolator<double>::ptr> getLt(vector<Trajectory::ptr>& trajectoryIpr);

	void doQuery();

	bool stop(double t, Interpolator<Q>::ptr& stopIpr);

	void resume(); //qStart为恢复点, 可以改为自动获取, 或留以作为位置误差判断

	bool isTrajectoryExist() const;

	Interpolator<Q>::ptr getQTrajectory() const;

	virtual ~MLBBPlanner();
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* MLBBPLANNER_H_ */
