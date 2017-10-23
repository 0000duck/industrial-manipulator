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
		Q qStart, Q qEnd)  :
			_dqLim(dqLim), _ddqLim(ddqLim),
			_vMax(vMaxLine), _aMax(aMaxLine), _h(hLine),
			_ikSolver(ikSolver), _serialLink(ikSolver->getRobot()),
			_qEnd(qEnd), _qStop(qStart), _config(_ikSolver->getConfig(qEnd))
{
	_size = _serialLink->getDOF();
	if (dqLim.size() != _size || ddqLim.size() != _size)
		throw ("错误<纯旋转规划>:　构造参数中数组的长度不一致！");
	/**> 检查dof */
	double dof = _serialLink->getDOF();
	if (dof != qStart.size() || dof != _qEnd.size())
		throw ("错误<纯旋转规划>: 查询的关节数值与机器人的自由度不符!");
	/**> 检查config参数 */
	if (_config != _ikSolver->getConfig(qStart))
		throw ("错误<纯旋转规划>: 初始和结束的Config不同!");
	Vector3D<double> startPos = _serialLink->getEndPosition(qStart);
	Vector3D<double> endPos = _serialLink->getEndPosition(qEnd);
	if (Vector3D<double>::distance(startPos, endPos) > 1e-12)
		throw("错误<纯旋转规划>: 开始和结束的位置不同!");
}

LineTrajectory::ptr RotationPlanner::query()
{
	Q start = _qStop;
	double assignedVelocity = _vMax;
	double assignedAcceleration = _aMax;
	Rotation3D<double> startRot = (_serialLink->getEndTransform(start)).getRotation();
	Rotation3D<double> endRot = (_serialLink->getEndTransform(_qEnd)).getRotation();
	Rotation3D<double> startToEndRot = startRot.inverse()*endRot;
	Quaternion startToEndQuat(startToEndRot);
	Quaternion::rotVar rvar = startToEndQuat.getRotationVariables();
	Vector3D<double> n = rvar.n;
	double rad = rvar.theta;
	Vector3D<double> startPos = _serialLink->getEndPosition(start);
	/**> 构造位置与姿态的线性插补器 角度为索引 rad作为插补长度 */
	auto posIpr = std::make_shared<FixedInterpolator<Vector3D<double> > >(startPos, rad);
	auto rotIpr = std::make_shared<RotationInterpolator>(startRot, n, rad);
	/**> 生成Trajectory */
	Trajectory::ptr trajectory(new Trajectory(std::make_pair(posIpr, rotIpr), _ikSolver, _config));
	/**> 直线平滑插补器_l(t) */
	int count = rad/_da + 1;
	count = (count < _countMin)? _countMin : count;

	/** 策略 */
	double velocity = trajectory->getMaxSpeed(count, _dqLim, _ddqLim, assignedVelocity);
	double acceleration = assignedAcceleration;

	SmoothMotionPlanner smPlanner;
	SequenceInterpolator<double>::ptr lt = smPlanner.query(rad, _h, acceleration, velocity, 0);
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
		throw( "警告<LinePlanner>: 尚未进行规划!\n");
	}
	Q qStop = _lineTrajectory->x(t);
	Vector3D<double> pStop = _serialLink->getEndPosition(qStop);
	Vector3D<double> pEnd = _serialLink->getEndPosition(_qEnd);
	double remainLength = Vector3D<double>::distance(pStop, pEnd);
	double s0 = _lineTrajectory->l(t);
	double v0 = _lineTrajectory->dl(t);
	double a0 = _lineTrajectory->ddl(t);
	SMPlannerEx planner;
	Interpolator<double>::ptr stopLt = planner.query_stop(s0, v0, a0, _h, _aMax);
	/**> 判断剩余距离是否足够停止 */
	if ((stopLt->end() - s0) >= remainLength)
	{
		cout << "错误<LinePlanner>: 距离不够, 无法停止!\n";
		return false;
	}
	stopIpr = std::make_shared<CompositeInterpolator<Q> > (_lineTrajectory->getTrajectory(), stopLt);
	_qStop = stopIpr->end();
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

Q RotationPlanner::findReachableEnd(Q start, Vector3D<double> n, std::shared_ptr<robot::ik::IKSolver> ikSolver, double da)
{
	Config config = ikSolver->getConfig(start);
	SerialLink::ptr robot = ikSolver->getRobot();
	const HTransform3D<double> startTran = robot->getEndTransform(start);
	const Vector3D<double> pos = startTran.getPosition();
	const Rotation3D<double> startRot = startTran.getRotation();
	double theta = 0;
	const double thetaMax = 2*M_PI; //限制一次最多旋转一圈
	Q end = start;
	do{
		theta += da; //下一个位置
		if (theta > thetaMax)
			break;
		try{
			end = ikSolver->solve(HTransform3D<double>(pos, startRot*(Quaternion(theta, n)).toRotation3D()), config)[0];
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
	return end;
}

RotationPlanner::~RotationPlanner() {
	// TODO Auto-generated destructor stub
}

} /* namespace pathplanner */
} /* namespace robot */
