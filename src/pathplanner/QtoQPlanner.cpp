/*
 * QtoQPlanner.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: a1994846931931
 */

#include "QtoQPlanner.h"

namespace robot {
namespace pathplanner {

QtoQPlanner::QtoQPlanner(Q dqLim, Q ddqLim,
		std::shared_ptr<robot::ik::IKSolver> ikSolver,
		Q start, Q qEnd)
: _dqLim(dqLim), _ddqLim(ddqLim),
  _ikSolver(ikSolver), _serialLink(ikSolver->getRobot()),
  _qStop(start), _qEnd(qEnd)
{

}

} /* namespace pathplanner */
} /* namespace robot */
