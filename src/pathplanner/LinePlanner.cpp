/*
 * LinePlanner.cpp
 *
 *  Created on: Sep 11, 2017
 *      Author: a1994846931931
 */

# include "LinePlanner.h"
# include "TimeOptimalPlanner.h"
# include "../common/printAdvance.h"
# include "../common/fileAdvance.h"
# include "../common/common.h"
# include "../math/Quaternion.h"
# include "../trajectory/CompositeInterpolator.h"
# include "../trajectory/Sampler.h"
# include "SmoothMotionPlanner.h"
# include "SMPlannerEx.h"
# include <memory>
# include <functional>
# include <algorithm>

using namespace robot::model;
using namespace robot::math;
using namespace robot::trajectory;
using namespace robot::common;

namespace robot {
namespace pathplanner {

LinePlanner::LinePlanner(Q dqLim, Q ddqLim,
		double vMaxLine, double aMaxLine, double hLine,
		std::shared_ptr<robot::ik::IKSolver> ikSolver,
		Q qStart, Q qEnd) :
	_vMax(vMaxLine), _aMax(aMaxLine), _h(hLine),
	_ikSolver(ikSolver), _serialLink(ikSolver->getRobot()), _qMin(_serialLink->getJointMin()), _qMax(_serialLink->getJointMax()),_dqLim(dqLim), _ddqLim(ddqLim),
	_qEnd(qEnd), _qStop(qStart), _config(_ikSolver->getConfig(qEnd))
{
	_size = _serialLink->getDOF();
	if (_qMin.size() != _size || _qMax.size() != _size || dqLim.size() != _size || ddqLim.size() != _size)
		throw ("错误<直线规划器>:　构造参数中数组的长度不一致！");
	/**> 检查dof */
	double dof = _serialLink->getDOF();
	if (dof != qStart.size() || dof != _qEnd.size())
		throw ("错误<直线规划>: 查询的关节数值与机器人的自由度不符!");
	/**> 检查config参数 */
	if (_config != _ikSolver->getConfig(qStart))
		throw ("错误<直线规划>: 初始和结束的Config不同!");
}

LineTrajectory::ptr LinePlanner::query()
{
	Q start = _qStop;
	double assignedVelocity = _vMax;
	double assignedAcceleration = _aMax;
	Vector3D<double> startPos = (_serialLink->getEndTransform(start)).getPosition();
	Vector3D<double> endPos = (_serialLink->getEndTransform(_qEnd)).getPosition();
	Vector3D<double> startToEndPos = startPos - endPos;
	double Length = startToEndPos.getLength();
	/**> 构造位置与姿态的线性插补器 l为索引 */
	LinearInterpolator<Vector3D<double> >::ptr posIpr(new LinearInterpolator<Vector3D<double> >(startPos, endPos, Length));
	LinearInterpolator<Rotation3D<double> >::ptr rotIpr(new LinearInterpolator<Rotation3D<double> >(
		_serialLink->getEndTransform(start).getRotation(), _serialLink->getEndTransform(_qEnd).getRotation(), Length));
	/**> 生成Trajectory */
	Trajectory::ptr trajectory(new Trajectory(std::make_pair(posIpr, rotIpr), _ikSolver, _config));
	/**> 直线平滑插补器_l(t) */
	int count = Length/_dl + 1;
	count = (count < _countMin)? _countMin : count;

	/** 最低速策略 */
	double velocity = trajectory->getMaxSpeed(count, _dqLim, _ddqLim, assignedVelocity);
	double acceleration = assignedAcceleration;
	SmoothMotionPlanner smPlanner;
	SequenceInterpolator<double>::ptr lt = smPlanner.query(Length, _h, acceleration, velocity, 0);

	/** 时间最优策略 */
//	vector<double> maxSpeed = trajectory->sampleMaxSpeed(count, _dqLim, _ddqLim, assignedVelocity);
//	vector<double> vl = robot::trajectory::Sampler<double>::linspace(0, Length, count);
//	saveDoublePath(to_string(getUTime()).c_str(), maxSpeed, vl);
//	std::function<double(double)> getMaxSpeed = [&](double l){
//		auto it = std::upper_bound(vl.begin(), vl.end(), l);
//		if (it == vl.end()) it--;
//		int idx = it - vl.begin();
//		if (0 == idx) idx = 1;
////		return (maxSpeed[idx] <= maxSpeed[idx - 1])? maxSpeed[idx]:maxSpeed[idx - 1]; //取最小
//		return (l - vl[idx - 1])/(vl[idx] - vl[idx - 1])*(maxSpeed[idx] - maxSpeed[idx - 1]) + maxSpeed[idx - 1]; //线性
//	};
//	SequenceInterpolator<double>::ptr lt;
//	lt = TimeOptimalPlanner::getOptimalLt(getMaxSpeed, Length, assignedVelocity, assignedAcceleration, _h, Length/(count - 1));
//	vector<double> vt = robot::trajectory::Sampler<double>::linspace(0, lt->duration(), 100);
//	vl = robot::trajectory::Sampler<double>::sample(vt, [&](double t){return lt->x(t);});
//	vector<double> vv = robot::trajectory::Sampler<double>::sample(vt, [&](double t){return lt->dx(t);});
//	saveDoublePath(to_string(getUTime()).c_str(), vv, vl);

	/**> 返回 */
	auto origin = std::make_pair(CompositeInterpolator<Vector3D<double> >::ptr(new CompositeInterpolator<Vector3D<double> >(posIpr, lt)),
			CompositeInterpolator<Rotation3D<double> >::ptr(new CompositeInterpolator<Rotation3D<double> >(rotIpr, lt)));
	LineTrajectory::ptr lineTrajectory(new LineTrajectory(origin, _ikSolver, _config, lt, trajectory));
	_lineTrajectory = lineTrajectory;
	return lineTrajectory;
}

void LinePlanner::doQuery()
{
	query();
}

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

void LinePlanner::resume()
{
	query();
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

Q LinePlanner::findReachableEnd(Q start, Vector3D<double> direction, std::shared_ptr<robot::ik::IKSolver> ikSolver, double dl)
{
	Config config = ikSolver->getConfig(start);
	SerialLink::ptr robot = ikSolver->getRobot();
	direction.doNormalize();
	direction = direction*dl;
	const HTransform3D<double> startTran = robot->getEndTransform(start);
	const Rotation3D<double> rot = startTran.getRotation();
	Vector3D<double> pos;
	pos = startTran.getPosition();
	Q end = start;
	do{
		pos += direction; //下一个位置
		try{
			vector<Q> result = ikSolver->solve(HTransform3D<double>(pos, rot), config);
			end = result[0];
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

} /* namespace pathplanner */
} /* namespace robot */
