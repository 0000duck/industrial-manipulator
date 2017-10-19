/*
 * LinePlanner.cpp
 *
 *  Created on: Sep 11, 2017
 *      Author: a1994846931931
 */

# include "LinePlanner.h"
# include "../common/printAdvance.h"
# include "../math/Quaternion.h"
# include "../trajectory/CompositeInterpolator.h"
# include "SmoothMotionPlanner.h"
# include "SMPlannerEx.h"
# include <memory>

using namespace robot::model;
using namespace robot::math;
using namespace robot::trajectory;
using namespace robot::common;

namespace robot {
namespace pathplanner {

LinePlanner::LinePlanner(Q dqLim, Q ddqLim,
		double vMaxLine, double aMaxLine, double hLine,
		std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink::ptr serialLink,
		Q qEnd) :
	_vMax(vMaxLine), _aMax(aMaxLine), _h(hLine),
	_ikSolver(ikSolver), _qMin(serialLink->getJointMin()), _qMax(serialLink->getJointMax()), _dqLim(dqLim), _ddqLim(ddqLim), _serialLink(serialLink),
	_qEnd(qEnd), _config(_ikSolver->getConfig(qEnd))
{
	_size = _serialLink->getDOF();
	if (_qMin.size() != _size || _qMax.size() != _size || dqLim.size() != _size || ddqLim.size() != _size)
		throw ("错误<直线规划器>:　构造参数中数组的长度不一致！");
}

LineTrajectory::ptr LinePlanner::query(const Q qStart)
{
	/**> 检查config参数 */
	if (_config != _ikSolver->getConfig(qStart))
		throw ("错误<直线规划>: 初始和结束的Config不同!");
	double assignedVelocity = _vMax;
	double assignedAcceleration = _aMax;
	/**> 检查dof */
	double dof = _serialLink->getDOF();
	if (dof != qStart.size() || dof != _qEnd.size())
		throw ("错误<直线规划>: 查询的关节数值与机器人的自由度不符!");
	Vector3D<double> startPos = (_serialLink->getEndTransform(qStart)).getPosition();
	Vector3D<double> endPos = (_serialLink->getEndTransform(_qEnd)).getPosition();
	Vector3D<double> startToEndPos = startPos - endPos;
	double Length = startToEndPos.getLength();
	/**> 构造位置与姿态的线性插补器 l为索引 */
	LinearInterpolator<Vector3D<double> >::ptr posIpr(new LinearInterpolator<Vector3D<double> >(startPos, endPos, Length));
	LinearInterpolator<Rotation3D<double> >::ptr rotIpr(new LinearInterpolator<Rotation3D<double> >(
		_serialLink->getEndTransform(qStart).getRotation(), _serialLink->getEndTransform(_qEnd).getRotation(), Length));
	/**> 生成Trajectory */
	Trajectory::ptr trajectory(new Trajectory(std::make_pair(posIpr, rotIpr), _ikSolver, _config));
	/**> 直线平滑插补器_l(t) */
	double sampledl = 0.01;
	int count = Length/sampledl + 1;

	/** 策略 */
	double velocity = trajectory->getMaxSpeed(count, _dqLim, _ddqLim, assignedVelocity);
	double acceleration = assignedAcceleration;

	SmoothMotionPlanner smPlanner;
	SequenceInterpolator<double>::ptr lt = smPlanner.query(Length, _h, acceleration, velocity, 0);
	/**> 返回 */
	auto origin = std::make_pair(CompositeInterpolator<Vector3D<double> >::ptr(new CompositeInterpolator<Vector3D<double> >(posIpr, lt)),
			CompositeInterpolator<Rotation3D<double> >::ptr(new CompositeInterpolator<Rotation3D<double> >(rotIpr, lt)));
	LineTrajectory::ptr lineTrajectory(new LineTrajectory(origin, _ikSolver, _config, lt, trajectory));
	_lineTrajectory = lineTrajectory;
	return lineTrajectory;
}

/**
 * @todo 转角的选择, 如果选择的旋转方向导致位置不可达到怎么办
 */
//LineTrajectory::ptr LinePlanner::query2(const Q qStart, const Q qEnd) const
//{
//	/**> 检查config参数 */
//	Config config = _ikSolver->getConfig(qStart);
//	if (config != _ikSolver->getConfig(qEnd))
//		throw ("错误<直线规划>: 初始和结束的Config不同!");
//	/**> 检查dof */
//	double dof = _serialLink->getDOF();
//	if (dof != qStart.size() || dof != qEnd.size())
//		throw ("错误<直线规划>: 查询的关节数值与机器人的自由度不符!");
//	/**> 构造直线以及角度的平滑插补器 */
//	Quaternion startQuaternion = _serialLink->getEndQuaternion(qStart);
//	Quaternion endQuaternion = _serialLink->getEndQuaternion(qEnd);
//	Quaternion startToEndQuat = startQuaternion.conjugate()*endQuaternion;
//	if (startToEndQuat.r() < 0)
//		startToEndQuat = -startToEndQuat; // 保证转角小于pi
//	Quaternion::rotVar rot = startToEndQuat.getRotationVariables();
//	Vector3D<double> startPos = (_serialLink->getEndTransform(qStart)).getPosition();
//	Vector3D<double> endPos = (_serialLink->getEndTransform(qEnd)).getPosition();
//	Vector3D<double> startToEndPos = startPos - endPos;
//	double Length = startToEndPos.getLength();
////	Vector3D<double> direction = startToEndPos/Length;
//	/**> 直线平滑插补器_l(t) */
//	SequenceInterpolator<double>::ptr lt = _smPlanner.query(Length, _hLine, _aMaxLine, _vMaxLine, 0);
//	/**> 角度平滑插补器_theta(t) */
//	SequenceInterpolator<double>::ptr tt = _smPlanner.query(rot.theta, _hAngle, _aMaxAngle, _vMaxAngle, 0);
//	/**> 统一插补器l(t)与theta(t)的时长 */
//	LinearCompositeInterpolator<double>::ptr mappedtt;
//	LinearCompositeInterpolator<double>::ptr mappedlt;
//	if ((lt->duration()) > (tt->duration()))
//	{
//		mappedtt = LinearCompositeInterpolator<double>::ptr(new LinearCompositeInterpolator<double>(tt, (tt->duration())/(lt->duration())));
//		mappedlt = LinearCompositeInterpolator<double>::ptr(new LinearCompositeInterpolator<double>(lt, 1));
//	}
//	else
//	{
//		mappedlt = LinearCompositeInterpolator<double>::ptr(new LinearCompositeInterpolator<double>(lt, (lt->duration())/(tt->duration())));
//		mappedtt = LinearCompositeInterpolator<double>::ptr(new LinearCompositeInterpolator<double>(tt, 1));
//	}
//	/**> 构造直线与角度的(位置与姿态)的线性插补器 */
//	LinearInterpolator<Vector3D<double> >::ptr posLinearInterpolator(new LinearInterpolator<Vector3D<double> >(startPos, endPos, Length));
//	LinearInterpolator<Rotation3D<double> >::ptr quatLinearInterpolator(new LinearInterpolator<Rotation3D<double> >(
//			startQuaternion.toRotation3D(), endQuaternion.toRotation3D(), rot.theta));
//	/**> 构造复合插补器, 构造直线与角度的(位置与姿态)的平滑插补器 */
//	CompositeInterpolator<Vector3D<double> >::ptr pos_t(new CompositeInterpolator<Vector3D<double> >(posLinearInterpolator, mappedlt));
//	CompositeInterpolator<Rotation3D<double> >::ptr quat_t(new CompositeInterpolator<Rotation3D<double> >(quatLinearInterpolator, mappedtt));
//	/**> 构造line插补器 */
//	std::pair<Interpolator<Vector3D<double> >::ptr, Interpolator<Rotation3D<double> >::ptr > endInterpolator(pos_t, quat_t);
//	LineTrajectory::ptr qInterpolator(new LineTrajectory(endInterpolator, _ikSolver, config, mappedlt, mappedtt)); /**> 直线Q插补器 */
//	/**> 约束检查, 若出现无法到达的采样点, 则抛出错误 */
//	int step = 1000;
//	double T = qInterpolator->duration();
//	double dt = T/(step - 1);
//	std::vector<Q> result;
//	State state(_size);
//	Q dqMax = _dqLim;
//	Q ddqMax = _ddqLim;
//	try{
//		for (double t=0; t<=T; t+=dt)
//		{
//			state = qInterpolator->getState(t);
//			for (int i=0; i<_size; i++)
//			{
//				if (dqMax(i) < fabs(state.getVelocity()[i]))
//					dqMax(i) = fabs(state.getVelocity()[i]);
//				if (ddqMax(i) < fabs(state.getAcceleration()[i]))
//					ddqMax(i) = fabs(state.getAcceleration()[i]);
//			}
//		}
//	}
//	catch(std::string& msg)
//	{
//		if (msg.find("无法进行逆解"))
//		{
//			throw(std::string("错误<直线规划>: 路径位置无法达到!\n") + std::string(msg));
//		}
//	}
//	/**> 若超出关节的速度与加速度约束, 则降低速度 */
//	Q kv = _dqLim/dqMax;
//	Q ka = _ddqLim/ddqMax;
//	double k = 1.0;
//	for (int i=0; i<_size; i++)
//	{
//		ka(i) = sqrt(ka[i]);
//	}
//	for (int i=0; i<_size; i++)
//	{
//		k = (k <= kv[i])? k:kv[i];
//		k = (k <= ka[i])? k:ka[i];
//	}
//	mappedlt->update(mappedlt->getFactor()*k);
//	mappedtt->update(mappedtt->getFactor()*k);
//	qInterpolator->doLengthAnalysis();
//	return qInterpolator;
//}

bool LinePlanner::stop(double t, Interpolator<Q>::ptr& stopIpr)
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

void LinePlanner::resume(const Q qStart)
{
	Q deltaQ = (qStart - _qStop);
	deltaQ.abs();
	if (deltaQ.getMax() > 0.01)
		cout << "警告<LinePlanner>: 恢复点与停止点的距离相差过大!\n";
	query(qStart);
}

bool LinePlanner::isTrajectoryExist() const
{
	if (_lineTrajectory.get() == NULL)
		return false;
	return true;
}

Interpolator<Q>::ptr LinePlanner::getQTrajectory() const
{
	return _lineTrajectory;
}

LinePlanner::~LinePlanner() {
}

} /* namespace pathplanner */
} /* namespace robot */
