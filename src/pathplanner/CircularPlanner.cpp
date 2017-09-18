/*
 * CircularPlanner.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: a1994846931931
 */

#include "CircularPlanner.h"

namespace robot {
namespace pathplanner {

CircularPlanner::CircularPlanner(Q qMin, Q qMax, Q dqLim, Q ddqLim,
		double vMaxLine, double aMaxLine, double hLine, double vMaxAngle, double aMaxAngle, double hAngle,
		std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink* serialLink) :
	_vMaxLine(vMaxLine), _aMaxLine(aMaxLine), _hLine(hLine), _vMaxAngle(vMaxLine), _aMaxAngle(aMaxLine), _hAngle(hLine),
	_ikSolver(ikSolver), _qMin(qMin), _qMax(qMax), _dqLim(dqLim), _ddqLim(ddqLim), _serialLink(serialLink)
{
	_size = qMin.size();
	if (qMax.size() != _size || qMax.size() != _size || dqLim.size() != _size || ddqLim.size() != _size)
		throw ("错误<圆弧规划器>:　构造参数中数组的长度不一致！");
}



} /* namespace pathplanner */
} /* namespace robot */
