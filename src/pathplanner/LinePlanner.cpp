/*
 * LinePlanner.cpp
 *
 *  Created on: Sep 11, 2017
 *      Author: a1994846931931
 */

# include "LinePlanner.h"
# include "../model/Config.h"
# include "../math/Quaternion.h"
# include "../trajectory/CompositeInterpolator.h"
# include "../trajectory/ConvertedInterpolator.h"
# include <memory>

using namespace robot::model;
using namespace robot::math;
using namespace robot::trajectory;
using namespace robot::common;

namespace robot {
namespace pathplanner {

LinePlanner::LinePlanner(Q qMin, Q qMax, Q dqLim, Q ddqLim,
		double vMaxLine, double aMaxLine, double hLine, double vMaxAngle, double aMaxAngle, double hAngle,
		std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink* serialLink) :
	_vMaxLine(vMaxLine), _aMaxLine(aMaxLine), _hLine(hLine), _vMaxAngle(vMaxLine), _aMaxAngle(aMaxLine), _hAngle(hLine),
	_ikSolver(ikSolver), _qMin(qMin), _qMax(qMax), _dqLim(dqLim), _ddqLim(ddqLim), _serialLink(serialLink)
{

}

/**
 * @brief todo 尚未完成采样, 约束检查
 * @param qStart
 * @param qEnd
 * @return
 */
Interpolator<Q>::ptr LinePlanner::query(Q qStart, Q qEnd)
{
	Config config = _serialLink->getConfig(qStart);
	if (config != _serialLink->getConfig(qEnd))
		throw ("错误: 初始和结束的Config不同!");
	double dof = _serialLink->getDOF();
	if (dof != qStart.size() || dof != qEnd.size())
		throw ("错误: 直线规划: 查询的关节数值与机器人的自由度不符!");
	Quaternion startQuaternion = _serialLink->getEndQuaternion(qStart);
	Quaternion endQuaternion = _serialLink->getEndQuaternion(qEnd);
	Quaternion startToEndQuat = startQuaternion.conjugate()*endQuaternion;
	if (startToEndQuat.r() < 0)
		startToEndQuat = -startToEndQuat;
	Quaternion::rotVar rot = startToEndQuat.getRotationVariables();
	Vector3D<double> startPos = (_serialLink->getEndTransform(qStart)).getPosition();
	Vector3D<double> endPos = (_serialLink->getEndTransform(qEnd)).getPosition();
	Vector3D<double> startToEndPos = startPos - endPos;
	double Length = startToEndPos.getLengh();
//	Vector3D<double> direction = startToEndPos/Length;
	/**> l(t) */
	SequenceInterpolator<double>::ptr lt = _smPlanner.query(Length, _hLine, _aMaxLine, _vMaxLine, 0);
	/**> theta(t) */
	SequenceInterpolator<double>::ptr tt = _smPlanner.query(rot.theta, _hAngle, _aMaxAngle, _vMaxAngle, 0);
	/**> 统一时间的l(t)与theta(t) */
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
	/**> 位姿插补器, 以长度和角度插补出位置和姿态 */
	LinearInterpolator<Vector3D<double> >::ptr posLinearInterpolator(new LinearInterpolator<Vector3D<double> >(startPos, endPos, Length));
	LinearInterpolator<Rotation3D<double> >::ptr quatLinearInterpolator(new LinearInterpolator<Rotation3D<double> >(
			startQuaternion.toRotation3D(), endQuaternion.toRotation3D(), rot.theta));
	/**> 位姿插补器, 以时间为变量插补出位置和姿态 */
	CompositeInterpolator<Vector3D<double> >::ptr pos_t(new CompositeInterpolator<Vector3D<double> >(posLinearInterpolator, mappedlt));
	CompositeInterpolator<Rotation3D<double> >::ptr quat_t(new CompositeInterpolator<Rotation3D<double> >(quatLinearInterpolator, mappedtt));
	/**> 合并为位姿插补器, 构造Q插补器 */
	std::pair<Interpolator<Vector3D<double> >::ptr, Interpolator<Rotation3D<double> >::ptr > endInterpolator(pos_t, quat_t);
	ikInterpolator::ptr qInterpolator(new ikInterpolator(endInterpolator, _ikSolver, config)); /**> Q插补器 */
	return qInterpolator;

}

LinePlanner::~LinePlanner() {
}

} /* namespace pathplanner */
} /* namespace robot */
