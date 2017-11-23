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
# include "../trajectory/ConvertedInterpolator.h"
# include "../trajectory/SequenceInterpolator.h"

using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

class QtoQPlanner {
public:
	using ptr = std::shared_ptr<Planner>;

	QtoQPlanner(Q dqLim, Q ddqLim,
			Q start, Q qEnd);

	Interpolator<Q>::ptr query();

	void doQuery();

	bool stop(double t, Interpolator<Q>::ptr& stopIpr);

	void resume(); //qStart为恢复点, 可以改为自动获取, 或留以作为位置误差判断

	bool isTrajectoryExist() const;

	Interpolator<Q>::ptr getQTrajectory() const;

	virtual ~QtoQPlanner(){}
private:
	Q _dqLim;
	Q _ddqLim;
	Q _qStop;
	Q _qEnd;
	const double _size;

	/**> 沿虚拟映射路径的速度加速度和加加速度 */
	double _v, _a, _h;

	std::vector<double> _k;

	std::vector<Interpolator<double>::ptr> _ltoqIpr;

	SequenceInterpolator<double>::ptr _lt;

	Interpolator<Q>::ptr _qIpr;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* QTOQPLANNER_H_ */
