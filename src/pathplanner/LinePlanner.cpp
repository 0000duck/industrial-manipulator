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

using namespace robot::model;
using namespace robot::math;

namespace robot {
namespace pathplanner {

LinePlanner::LinePlanner() {
	// TODO Auto-generated constructor stub

}

Interpolator<Q>* LinePlanner::query(Q qStart, Q qEnd)
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
	Vector3D<double> direction = startToEndPos/Length;

	/**> l(t) */
	Interpolator<double>* lt = _smPlanner.query(Length, _hLine, _aMaxLine, _vMaxLine, 0);
	/**> theta(t) */
	Interpolator<double>* tt = _smPlanner.query(rot.theta, _hAngle, _aMaxAngle, _vMaxAngle, 0);
	/**> 统一时间 */
	LinearCompositeInterpolator<double>* mappedtt;
	LinearCompositeInterpolator<double>* mappedlt;
	double duration;
	if ((lt->duration()) > (tt->duration()))
	{
		mappedtt = new LinearCompositeInterpolator<double>(tt, (tt->duration())/(lt->duration()));
		mappedlt = new LinearCompositeInterpolator<double>(lt, 1);
		duration = lt->duration();
	}
	else
	{
		mappedlt = new LinearCompositeInterpolator<double>(lt, (lt->duration())/(tt->duration()));
		mappedtt = new LinearCompositeInterpolator<double>(tt, 1);
		duration = tt->duration();
	}
	_lcInterpolatorList.push_back(mappedlt);
	_lcInterpolatorList.push_back(mappedtt);
	/**> 位姿插补器 */
	LinearInterpolator<Vector3D<double> >* posLinearInterpolator = new LinearInterpolator<Vector3D<double> >(startPos, endPos, Length);
	LinearInterpolator<Rotation3D<double> >* angleLinearInterpolator = new LinearInterpolator<Rotation3D<double> >(
			startQuaternion.toRotation3D(), endQuaternion.toRotation3D(), rot.theta);
	_lvInterpolatorList.push_back(posLinearInterpolator);
	_lrInterpolatorList.push_back(angleLinearInterpolator);
	std::pair<LinearInterpolator<Vector3D<double> >*, LinearInterpolator<Rotation3D<double> >*> endInterpolator;

}

LinePlanner::~LinePlanner() {
	// TODO Auto-generated destructor stub
}

} /* namespace pathplanner */
} /* namespace robot */
