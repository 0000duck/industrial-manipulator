/*
 * QtoQPlanner.h
 *
 *  Created on: Oct 27, 2017
 *      Author: a1994846931931
 */

#ifndef QTOQPLANNER_H_
#define QTOQPLANNER_H_

# include "Planner.h"
# include "../ik/IKSolver.h"

namespace robot {
namespace pathplanner {

class QtoQPlanner {
public:
	using ptr = std::shared_ptr<Planner>;

	QtoQPlanner(Q dqLim, Q ddqLim,
			std::shared_ptr<robot::ik::IKSolver> ikSolver,
			Q start, Q qEnd);

	void query();

	void doQuery();
	bool stop(double t, Interpolator<Q>::ptr& stopIpr);
	void resume(); //qStart为恢复点, 可以改为自动获取, 或留以作为位置误差判断
	bool isTrajectoryExist() const;
	Interpolator<Q>::ptr getQTrajectory() const;

	virtual ~QtoQPlanner(){}
private:
	Q _dqLim;
	Q _ddqLim;
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;
	SerialLink::ptr  _serialLink;
	Q _qStop;
	Q _qEnd;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* QTOQPLANNER_H_ */
