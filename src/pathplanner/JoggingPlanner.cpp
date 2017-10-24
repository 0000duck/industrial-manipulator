/*
 * JoggingPlanner.cpp
 *
 *  Created on: Oct 19, 2017
 *      Author: a1994846931931
 */

#include "JoggingPlanner.h"
# include "LinePlanner.h"
# include "RotationPlanner.h"

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

Planner::ptr JoggingPlanner::jogX(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot)
{
	Vector3D<double> direction(1, 0, 0);
	if (!isPositive)
		direction = -direction;
	direction = rot*direction;
	return planLine(current, farEnd, direction);
}

Planner::ptr JoggingPlanner::jogY(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot)
{
	Vector3D<double> direction(0, 1, 0);
	if (!isPositive)
		direction = -direction;
	direction = rot*direction;
	return planLine(current, farEnd, direction);
}

Planner::ptr JoggingPlanner::jogZ(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot)
{
	Vector3D<double> direction(0, 0, 1);
	if (!isPositive)
		direction = -direction;
	direction = rot*direction;
	return planLine(current, farEnd, direction);
}

Planner::ptr JoggingPlanner::jogRX(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot) //末端的位置由机器人模型指定
{
	Vector3D<double> direction(1, 0, 0);
	if (!isPositive)
		direction = -direction;
	direction = rot*direction;
	return planRotation(current, farEnd, direction);
}

Planner::ptr JoggingPlanner::jogRY(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot)
{
	Vector3D<double> direction(0, 1, 0);
	if (!isPositive)
		direction = -direction;
	direction = rot*direction;
	return planRotation(current, farEnd, direction);
}

Planner::ptr JoggingPlanner::jogRZ(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot)
{
	Vector3D<double> direction(0, 0, 1);
	if (!isPositive)
		direction = -direction;
	direction = rot*direction;
	return planRotation(current, farEnd, direction);
}

JoggingPlanner::~JoggingPlanner()
{

}

/*** private ***/

Planner::ptr JoggingPlanner::planLine(Q current, Q &farEnd, Vector3D<double> direction)
{
	Q end = LinePlanner::findReachableEnd(current, direction, _ikSolver); //默认采样长度
	auto planner = std::make_shared<LinePlanner>(_dqLim, _ddqLim, _vLine, _aLine, _jLine, _ikSolver, current, end); //返回的是已近做好规划的规划器, 可以直接添加到运动堆栈里面
	try{
		planner->query();
		farEnd = planner->getQTrajectory()->end();
	}
	catch(std::string& msg)
	{
		cout << msg << endl;
	}
	return planner;
}

Planner::ptr JoggingPlanner::planRotation(Q current, Q &farEnd, Vector3D<double> direction)
{
	double theta = RotationPlanner::findReachableTheta(current, direction, _ikSolver); //默认采样长度
	auto planner = std::make_shared<RotationPlanner>(_dqLim, _ddqLim, _vAngle, _aAngle, _jAngle, _ikSolver, current, direction, theta); //返回的是已近做好规划的规划器, 可以直接添加到运动堆栈里面
	try{
		planner->query();
		farEnd = planner->getQTrajectory()->end();
	}
	catch(std::string& msg)
	{
		cout << msg << endl;
	}
	return planner;
}

} /* namespace pathplanner */
} /* namespace robot */
