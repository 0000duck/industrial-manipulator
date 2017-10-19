/*
 * JoggingPlanner.cpp
 *
 *  Created on: Oct 19, 2017
 *      Author: a1994846931931
 */

#include "JoggingPlanner.h"
# include "LinePlanner.h"

namespace robot {
namespace pathplanner {

JoggingPlanner::JoggingPlanner(std::shared_ptr<robot::ik::IKSolver> ikSolver, vector<double> constraints, Q dqLim, Q ddqLim)
{
	_ikSolver = ikSolver;
	_vLine = constraints[0];
	_aLine = constraints[1];
	_jLine = constraints[2];
	_vAngle = constraints[3];
	_aAngle = constraints[4];
	_jAngle = constraints[5];
	_dqLim = dqLim;
	_ddqLim = ddqLim;
}

Planner::ptr JoggingPlanner::jogX(Q current, Q &farEnd, Rotation3D<double> rot)
{
	Vector3D<double> direction(1, 0, 0);
	direction = rot*direction;
	farEnd = LinePlanner::findReachableEnd(current, direction, _ikSolver); //m默认采样长度
	current.print();
	farEnd.print();
	auto planner = std::make_shared<LinePlanner>(_dqLim, _ddqLim, _vLine, _aLine, _jLine, _ikSolver, _ikSolver->getRobot(), farEnd); //返回的是已近做好规划的规划器, 可以直接添加到运动堆栈里面
	try{
		planner->query(current);
	}
	catch(std::string& msg)
	{
		cout << msg << endl;
	}
	return planner;
}

JoggingPlanner::~JoggingPlanner()
{

}

} /* namespace pathplanner */
} /* namespace robot */
