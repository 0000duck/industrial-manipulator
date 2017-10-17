/*
 * MotionStack.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: a1994846931931
 */

#include "MotionStack.h"

namespace robot {
namespace simulation {

MotionStack::MotionStack() {
	_id = 0;
	_status = 0;
}

int MotionStack::addPlanner(Planner::ptr planner)
{
	if ( ! planner->isTrajectoryExist())
		return 3;
	if ((int)_motionQueue.size() >= _maxDataCount)
		return 1;
	if ((int)_motionQueue.size() > 0)
	{
		Q end = _motionQueue.back().planner->getQTrajectory()->end();
		Q start = planner->getQTrajectory()->start();
		Q delta = end - start;
		delta.abs();
		if (delta.getMax() > 0.0001)
			return 2;
	}
	_motionQueue.push(motionData{_id++, planner, planner->getQTrajectory()->duration()});
	if (_status == 0)
		_status = 1;
	return 0;
}

MotionStack::~MotionStack() {
	// TODO Auto-generated destructor stub
}

} /* namespace simulation */
} /* namespace robot */
