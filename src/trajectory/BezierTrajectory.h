/*
 * BezierTrajectory.h
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */

#ifndef BEZIERTRAJECTORY_H_
#define BEZIERTRAJECTORY_H_

# include "Trajectory.h"

namespace robot {
namespace trajectory {

class BezierTrajectory{
public:
	BezierTrajectory();
	virtual ~BezierTrajectory();
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* BEZIERTRAJECTORY_H_ */
