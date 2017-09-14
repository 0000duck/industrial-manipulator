/*
 * QBlend.cpp
 *
 *  Created on: Sep 13, 2017
 *      Author: a1994846931931
 */

# include "QBlend.h"
# include "../common/printAdvance.h"
# include "../trajectory/PolynomialInterpolator.h"
# include "../trajectory/ConvertedInterpolator.h"
# include "../trajectory/CompositeInterpolator.h"

using robot::math::Q;
using std::vector;
using robot::kinematic::State;
using namespace robot::trajectory;
using namespace robot::common;

namespace robot {
namespace pathplanner {

QBlend::QBlend(Q aMax, Q vMax, Q qMin, Q qMax)
{
	_aMax = aMax;
	_vMax = vMax;
	_qMin = qMin;
	_qMax = qMax;
	_size = _aMax.size();
	if (_aMax.size() != _size || _vMax.size() != _size)
		throw ("错误<Q混合规划器>: 点对点规划器必须由同样大小的Q进行构造!");
}

Interpolator<Q>::ptr QBlend::query(State qStart, State qEnd, double maxDuration)
{
	const Q startQ = qStart.getAngle();
	const Q startVel = qStart.getVelocity();
	const Q startAcc = qStart.getAcceleration();
	const Q endQ = qEnd.getAngle();
	const Q endVel = qEnd.getVelocity();
	const Q endAcc = qEnd.getAcceleration();

	vector<Interpolator<double>::ptr > polynomialInterpolators;
	vector<Interpolator<double>::ptr > mappedPolyIpr;
	/**> 计算时长 */
	// duration = maxDuration;
	/**> 五次多项式规划每个关节 */
	for (int i=0; i<_size; i++)
	{
		polynomialInterpolators.push_back(PolynomialInterpolator5<double>::make(
				0, maxDuration, startQ[i], startVel[i], startAcc[i], endQ[i], endVel[i], endAcc[i], maxDuration)); /** @todo 添加五次插补器 */
	}
	/**> 打包成Q插补器 */
	std::shared_ptr<ConvertedInterpolator<std::vector<Interpolator<double>::ptr > , robot::math::Q> > QIpr(
				new ConvertedInterpolator<std::vector<Interpolator<double>::ptr > , robot::math::Q>(polynomialInterpolators));
	/**> 检查约束 */
	int step = 1000;
	std::pair<Q, Q> qLim = QIpr->getLimQ(step);
	if (qLim.first < _qMin || qLim.second > _qMax)
		throw ("错误<Q混合规划器>: 无法混合, 关节超出限位");
	Q vMax = QIpr->getMaxdQ(step);
	Q aMax = QIpr->getMaxddQ(step);
	Q kv = vMax/_vMax;
	Q ka = aMax/_aMax;
	double k = kv[0];
	for (int i=0; i<_size; i++)
	{
		ka(i) = sqrt(ka[i]);
	}
	for (int i=0; i<_size; i++)
	{
		k = (k >= kv[i])? k:kv[i];
		k = (k >= ka[i])? k:ka[i];
	}
	/**> 速度过慢, 迭代处理 */
	if (k < _kMin)
		return this->query(qStart, qEnd, maxDuration*2*k/(1.0 + _kMin));
	/**> 速度过快, 迭代处理 */
	if (k > 1.0)
		return this->query(qStart, qEnd, maxDuration*2*k/(1.0 + _kMin));
	/**> 速度适中, 返回 */
	return QIpr;
}

QBlend::~QBlend() {
}

} /* namespace pathplanner */
} /* namespace robot */
