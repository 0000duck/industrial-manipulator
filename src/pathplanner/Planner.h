/*
 * Planner.h
 *
 *  Created on: Oct 17, 2017
 *      Author: a1994846931931
 */

#ifndef PLANNER_H_
#define PLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"
# include "../kinematics/State.h"
# include <memory>

using robot::trajectory::Interpolator;
using robot::math::Q;
using robot::kinematic::State;

namespace robot {
namespace pathplanner {

class Planner {
public:
	using ptr = std::shared_ptr<Planner>;

	Planner();
	virtual ~Planner();

	virtual void doQuery() = 0;
	virtual bool stop(double t, Interpolator<Q>::ptr& stopIpr) = 0;
	virtual void resume() = 0;
	virtual bool isTrajectoryExist() const = 0;
	virtual Interpolator<Q>::ptr getQTrajectory() const = 0;
protected:
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* PLANNER_H_ */
