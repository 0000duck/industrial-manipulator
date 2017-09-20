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
# include <memory>

using namespace robot::model;
using namespace robot::math;
using namespace robot::trajectory;
using namespace robot::common;

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
		throw ("错误<圆弧规划>:　构造参数中数组的长度不一致！");
}

Interpolator<Q>::ptr CircularPlanner::query(const Q qStart, const Q qIntermediate, const Q qEnd) const
{
	/**> 检查config参数 */
	Config config = _serialLink->getConfig(qStart);
	if (config != _serialLink->getConfig(qEnd) || config != _serialLink->getConfig(qIntermediate))
		throw ("错误<圆弧规划>: 初始, 中间点和结束的Config不同!");
	/**> 检查dof */
	double dof = _serialLink->getDOF();
	if (dof != qStart.size() || dof != qEnd.size() || dof != qIntermediate.size())
		throw ("错误<圆弧规划>: 查询的关节数值与机器人的自由度不符!");
	/**> 构造直线以及角度的平滑插补器 */
	Quaternion startQuaternion = _serialLink->getEndQuaternion(qStart);
	Quaternion endQuaternion = _serialLink->getEndQuaternion(qEnd);
	Quaternion startToEndQuat = startQuaternion.conjugate()*endQuaternion;
	if (startToEndQuat.r() < 0)
		startToEndQuat = -startToEndQuat;
	Quaternion::rotVar rot = startToEndQuat.getRotationVariables();
	Vector3D<double> startPos = (_serialLink->getEndTransform(qStart)).getPosition();
	Vector3D<double> intermediatePos = (_serialLink->getEndTransform(qIntermediate)).getPosition();
	Vector3D<double> endPos = (_serialLink->getEndTransform(qEnd)).getPosition();
	/**> 构造圆弧与角度的(位置与姿态)的线性插补器 */
	CircularInterpolator<Vector3D<double> >::ptr posLinearInterpolator(new CircularInterpolator<Vector3D<double> >(startPos, intermediatePos, endPos));
	LinearInterpolator<Rotation3D<double> >::ptr quatLinearInterpolator(new LinearInterpolator<Rotation3D<double> >(
			startQuaternion.toRotation3D(), endQuaternion.toRotation3D(), rot.theta));
	double Length = posLinearInterpolator->getLength();
	/**> 平滑插补器_l(t) */
	SequenceInterpolator<double>::ptr lt = _smPlanner.query(Length, _hLine, _aMaxLine, _vMaxLine, 0);
	/**> 角度平滑插补器_theta(t) */
	SequenceInterpolator<double>::ptr tt = _smPlanner.query(rot.theta, _hAngle, _aMaxAngle, _vMaxAngle, 0);
	/**> 统一插补器l(t)与theta(t)的时长 */
	LinearCompositeInterpolator<double>::ptr mappedtt;
	LinearCompositeInterpolator<double>::ptr mappedlt;
	if ((lt->duration()) > (tt->duration()))
	{
		mappedtt = LinearCompositeInterpolator<double>::ptr(new LinearCompositeInterpolator<double>(tt, (tt->duration())/(lt->duration())));
		mappedlt = LinearCompositeInterpolator<double>::ptr(new LinearCompositeInterpolator<double>(lt, 1));
	}
	else
	{
		mappedlt = LinearCompositeInterpolator<double>::ptr(new LinearCompositeInterpolator<double>(lt, (lt->duration())/(tt->duration())));
		mappedtt = LinearCompositeInterpolator<double>::ptr(new LinearCompositeInterpolator<double>(tt, 1));
	}
	/**> 构造复合插补器, 构造直线与角度的(位置与姿态)的平滑插补器 */
	CompositeInterpolator<Vector3D<double> >::ptr pos_t(new CompositeInterpolator<Vector3D<double> >(posLinearInterpolator, mappedlt));
	CompositeInterpolator<Rotation3D<double> >::ptr quat_t(new CompositeInterpolator<Rotation3D<double> >(quatLinearInterpolator, mappedtt));
	/**> 构造ik插补器 */
	std::pair<Interpolator<Vector3D<double> >::ptr, Interpolator<Rotation3D<double> >::ptr > endInterpolator(pos_t, quat_t);
	ikInterpolator::ptr qInterpolator(new ikInterpolator(endInterpolator, _ikSolver, config)); /**> Q插补器 */
	/**> 约束检查, 若出现无法到达的采样点, 则抛出错误 */
	int step = 1000;
	double T = qInterpolator->duration();
	double dt = T/(step - 1);
	std::vector<Q> result;
	State state(_size);
	Q dqMax = _dqLim;
	Q ddqMax = _ddqLim;
	try{
		for (double t=0; t<=T; t+=dt)
		{
			state = qInterpolator->getState(t);
			for (int i=0; i<_size; i++)
			{
				if (dqMax(i) < fabs(state.getVelocity()[i]))
					dqMax(i) = fabs(state.getVelocity()[i]);
				if (ddqMax(i) < fabs(state.getAcceleration()[i]))
					ddqMax(i) = fabs(state.getAcceleration()[i]);
			}
		}
	}
	catch(std::string& msg)
	{
		if (msg.find("无法进行逆解"))
		{
			throw(std::string("错误<直线规划>: 路径位置无法达到!\n") + std::string(msg));
		}
	}
	/**> 若超出关节的速度与加速度约束, 则降低速度 */
	Q kv = _dqLim/dqMax;
	Q ka = _ddqLim/ddqMax;
	double k = 1.0;
	for (int i=0; i<_size; i++)
	{
		ka(i) = sqrt(ka[i]);
	}
	for (int i=0; i<_size; i++)
	{
		k = (k <= kv[i])? k:kv[i];
		k = (k <= ka[i])? k:ka[i];
	}
	mappedlt->update(mappedlt->getFactor()*k);
	mappedtt->update(mappedtt->getFactor()*k);
	return qInterpolator;
}

} /* namespace pathplanner */
} /* namespace robot */
