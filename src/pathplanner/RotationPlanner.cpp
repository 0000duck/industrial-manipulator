/*
 * RotationPlanner.cpp
 *
 *  Created on: Oct 23, 2017
 *      Author: a1994846931931
 */

#include "RotationPlanner.h"
# include "SmoothMotionPlanner.h"
# include "SMPlannerEx.h"

using robot::model::Config;

namespace robot {
namespace pathplanner {

RotationPlanner::RotationPlanner(Q dqLim, Q ddqLim,
		double vMaxLine, double aMaxLine, double hLine,
		std::shared_ptr<robot::ik::IKSolver> ikSolver,
		Q qStart, Vector3D<double> n, double theta)  :
			_dqLim(dqLim), _ddqLim(ddqLim),
			_vMax(vMaxLine), _aMax(aMaxLine), _h(hLine),
			_ikSolver(ikSolver), _serialLink(ikSolver->getRobot()),
			_n(n), _theta(theta), _qStop(qStart), _config(_ikSolver->getConfig(qStart))
{
	_size = _serialLink->getDOF();
	if (dqLim.size() != _size || ddqLim.size() != _size)
		throw ("错误<纯旋转规划>:　构造参数中数组的长度不一致！");
}

LineTrajectory::ptr RotationPlanner::query()
{
	Q start = _qStop;
	double assignedVelocity = _vMax;
	double assignedAcceleration = _aMax;
	Rotation3D<double> startRot = (_serialLink->getEndTransform(start)).getRotation();
	Vector3D<double> startPos = _serialLink->getEndPosition(start);
	/**> 构造位置与姿态的线性插补器 角度为索引 _theta作为插补长度 */
	auto rotIpr = std::make_shared<RotationInterpolator>(startRot, _n, _theta);
	auto posIpr = std::make_shared<FixedInterpolator<Vector3D<double> > >(startPos, rotIpr->duration()); //当rad=0时, 实际的插补时长由RotationInterpolator内部指定
	/**> 生成Trajectory */
	Trajectory::ptr trajectory(new Trajectory(std::make_pair(posIpr, rotIpr), _ikSolver, _config));
	/**> 直线平滑插补器_l(t) */
	int count = _theta/_da + 1;
	count = (count < _countMin)? _countMin : count;

	/** 策略 */
	double velocity = trajectory->getMaxSpeed(count, _dqLim, _ddqLim, assignedVelocity);
	double acceleration = assignedAcceleration;

	SmoothMotionPlanner smPlanner;
	SequenceInterpolator<double>::ptr lt = smPlanner.query(rotIpr->duration(), _h, acceleration, velocity, 0);
	/**> 返回 */
	auto origin = std::make_pair(CompositeInterpolator<Vector3D<double> >::ptr(new CompositeInterpolator<Vector3D<double> >(posIpr, lt)),
			CompositeInterpolator<Rotation3D<double> >::ptr(new CompositeInterpolator<Rotation3D<double> >(rotIpr, lt)));
	LineTrajectory::ptr lineTrajectory(new LineTrajectory(origin, _ikSolver, _config, lt, trajectory));
	_lineTrajectory = lineTrajectory;
	return lineTrajectory;
}

void RotationPlanner::doQuery()
{
	query();
}

bool RotationPlanner::stop(double t, Interpolator<Q>::ptr& stopIpr)
{
	if (_lineTrajectory.get() == NULL)
	{
		throw( "警告<纯旋转规划>: 尚未进行规划!");
	}
	double S = _lineTrajectory->l(_lineTrajectory->duration());
	double s0 = _lineTrajectory->l(t);
	double remainAngle = S - s0;
	double v0 = _lineTrajectory->dl(t);
	double a0 = _lineTrajectory->ddl(t);
	SMPlannerEx planner;
	Interpolator<double>::ptr stopLt = planner.query_stop(s0, v0, a0, _h, _aMax);
	/**> 判断剩余距离是否足够停止 */
	if ((stopLt->end() - s0) >= remainAngle)
	{
		cout << "错误<纯旋转规划>: 距离不够, 无法停止!\n";
		return false;
	}
	stopIpr = std::make_shared<CompositeInterpolator<Q> > (_lineTrajectory->getTrajectory(), stopLt);
	_qStop = stopIpr->end();
	_theta -= stopLt->end();
	return true;
}

void RotationPlanner::resume()
{
	query();
}

bool RotationPlanner::isTrajectoryExist() const
{
	if (_lineTrajectory.get() == NULL)
		return false;
	return true;
}

Interpolator<Q>::ptr RotationPlanner::getQTrajectory() const
{
	return _lineTrajectory;
}

double RotationPlanner::findReachableTheta(Q start, Vector3D<double> n, std::shared_ptr<robot::ik::IKSolver> ikSolver, double da)
{
	Config config = ikSolver->getConfig(start);
	SerialLink::ptr robot = ikSolver->getRobot();
	const HTransform3D<double> startTran = robot->getEndTransform(start);
	const Vector3D<double> pos = startTran.getPosition();
	const Rotation3D<double> startRot = startTran.getRotation();
	double theta = 0;
	const double thetaMax = 2*M_PI; //限制一次最多旋转一圈
	Q result; //用来保存结果,做奇异点检查
	do{
		theta += da; //下一个位置 ////////////////////
		if (theta > thetaMax)
			break;
		try{
			result = ikSolver->solve(HTransform3D<double>(pos, startRot*(Quaternion(theta, n)).toRotation3D()), config)[0];
			int singular;
			if ((singular = ikSolver->singularJudge(result)) != 0)
			{
				printf("奇异点: %02X\n", singular);
				break;
			}
		}
		catch(char const*)
		{
			break;
		}
		catch(std::string &)
		{
			break;
		}
	}
	while(true);
	theta -= da;
	return theta;
}

RotationPlanner::~RotationPlanner() {
	// TODO Auto-generated destructor stub
}

} /* namespace pathplanner */
} /* namespace robot */
