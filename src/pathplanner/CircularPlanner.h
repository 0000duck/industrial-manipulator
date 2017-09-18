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
# include <memory>

using robot::math::Q;
using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

class CircularPlanner {
public:
	CircularPlanner(Q qMin, Q qMax, Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine, double vMaxAngle, double aMaxAngle, double hAngle,
			std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink* serialLink);
	Interpolator<Q>::ptr query(const Q qStart, const Q intermediate, const Q qEnd) const;
	virtual ~CircularPlanner(){}
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* CIRCULARPLANNER_H_ */
