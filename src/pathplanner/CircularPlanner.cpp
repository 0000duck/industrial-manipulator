/*
 * CircularPlanner.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: a1994846931931
 */

# include "CircularPlanner.h"
# include "../common/printAdvance.h"
# include "../model/Config.h"
# include "../math/Quaternion.h"
# include "../trajectory/CompositeInterpolator.h"
# include "../trajectory/ConvertedInterpolator.h"
# include "../trajectory/CircularInterpolator.h"
# include "../trajectory/Trajectory.h"
# include "../trajectory/CircularTrajectory.h"
# include "SMPlannerEx.h"
# include <memory>

using namespace robot::model;
using namespace robot::math;
using namespace robot::trajectory;
using namespace robot::common;

namespace robot {
namespace pathplanner {

CircularPlanner::CircularPlanner(Q dqLim, Q ddqLim,
		double vMaxLine, double aMaxLine, double hLine,
		std::shared_ptr<robot::ik::IKSolver> ikSolver,
		const Q qStart, const Q qIntermediate, const Q qEnd) :
	_vMax(vMaxLine), _aMax(aMaxLine), _h(hLine),
	_ikSolver(ikSolver), _serialLink(ikSolver->getRobot()), _qMin(_serialLink->getJointMin()), _qMax(_serialLink->getJointMax()), _dqLim(dqLim), _ddqLim(ddqLim),
	_qStop(qStart), _qIntermediate(qIntermediate), _qEnd(qEnd)
{
	_size = _serialLink->getDOF();
	if (_qMax.size() != _size || _qMax.size() != _size || dqLim.size() != _size || ddqLim.size() != _size || _size != _qIntermediate.size() || _size != _qEnd.size())
		throw ("错误<圆弧规划>:　构造参数中数组的长度不一致！");

	/**> 检查config参数 */
	_config = _ikSolver->getConfig(qIntermediate);
	if (_config != _ikSolver->getConfig(qEnd))
		throw ("错误<圆弧规划>: 中间点和结束点的Config不同!");
	/**> 检查dof */
	if (_size != qStart.size())
		throw ("错误<圆弧规划>: 查询的关节数值与机器人的自由度不符!");
}

CircularTrajectory::ptr CircularPlanner::query()
{
	Q qStart = _qStop;
	double assignedVelocity = _vMax;
	double assignedAcceleration = _aMax;
	Vector3D<double> startPos = (_serialLink->getEndTransform(qStart)).getPosition();
	Vector3D<double> intermediatePos = (_serialLink->getEndTransform(_qIntermediate)).getPosition();
	Vector3D<double> endPos = (_serialLink->getEndTransform(_qEnd)).getPosition();
	/**> 构造圆弧位置与姿态的线性插补器 l为索引 */
	CircularInterpolator<Vector3D<double> >::ptr posIpr(new CircularInterpolator<Vector3D<double> >(startPos, intermediatePos, endPos));
	double Length = posIpr->getLength();
	LinearInterpolator<Rotation3D<double> >::ptr rotIpr(new LinearInterpolator<Rotation3D<double> >(
		_serialLink->getEndTransform(qStart).getRotation(), _serialLink->getEndTransform(_qEnd).getRotation(), Length));
	/**> 生成Trajectory */
	Trajectory::ptr trajectory(new Trajectory(std::make_pair(posIpr, rotIpr), _ikSolver, _config));
	/**> 直线平滑插补器_l(t) */
	int count = Length/_dl + 1;
	count = (count < _countMin)? _countMin : count;
	/** 策略 */
	double velocity = trajectory->getMaxSpeed(count, _dqLim, _ddqLim, assignedVelocity);
	double acceleration = assignedAcceleration;
	SmoothMotionPlanner smPlanner;
	SequenceInterpolator<double>::ptr lt = smPlanner.query(Length, _h, acceleration, velocity, 0);
	/**> 返回 */
	auto origin = std::make_pair(CompositeInterpolator<Vector3D<double> >::ptr(new CompositeInterpolator<Vector3D<double> >(posIpr, lt)),
			CompositeInterpolator<Rotation3D<double> >::ptr(new CompositeInterpolator<Rotation3D<double> >(rotIpr, lt)));
	CircularTrajectory::ptr circularTrajectory(new CircularTrajectory(origin, _ikSolver, _config, lt, trajectory));
	_circularTrajectory = circularTrajectory;
	return circularTrajectory;
}

void CircularPlanner::doQuery()
{
	query();
}

bool CircularPlanner::stop(double t, Interpolator<Q>::ptr& stopIpr)
{
	if (_circularTrajectory.get() == NULL)
	{
		throw( "警告<CircularPlanner>: 尚未进行规划!\n");
	}
	double s0 = _circularTrajectory->l(t);
	double v0 = _circularTrajectory->dl(t);
	double a0 = _circularTrajectory->ddl(t);
	double S = _circularTrajectory->l(_circularTrajectory->duration());
	double remainLength = S - s0;
	SMPlannerEx planner;
	Interpolator<double>::ptr stopLt = planner.query_stop(s0, v0, a0, _h, _aMax);
	/**> 判断剩余距离是否足够停止 */
	if ((stopLt->end() -s0) >= remainLength)
	{
		cout << "错误<CircularPlanner>: 距离不够, 无法停止!\n";
		return false;
	}
	Trajectory::ptr originalTrajectory = _circularTrajectory->getTrajectory();
	stopIpr = std::make_shared<CompositeInterpolator<Q> > (originalTrajectory, stopLt);
	double s1 = stopLt->end();
	cout << "s1 = " << s1 << " s0 = " << s0 << " S = " << S << endl;
	_qIntermediate = originalTrajectory->x((s1 + S)/2.0); //重新计算中间点
	_qStop = stopIpr->end();
	return true;
}

void CircularPlanner::resume()
{
	query();
}

bool CircularPlanner::isTrajectoryExist() const
{
	if (_circularTrajectory.get() == NULL)
		return false;
	return true;
}
Interpolator<Q>::ptr CircularPlanner::getQTrajectory() const
{
	return _circularTrajectory;
}

} /* namespace pathplanner */
} /* namespace robot */
