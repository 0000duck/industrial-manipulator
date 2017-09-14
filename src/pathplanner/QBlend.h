/*
 * QBlend.h
 *
 *  Created on: Sep 13, 2017
 *      Author: a1994846931931
 */

#ifndef QBLEND_H_
#define QBLEND_H_

# include "../math/Q.h"
# include "../trajectory/Interpolator.h"
# include "../kinematics/State.h"

using robot::math::Q;
using std::vector;
using robot::kinematic::State;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/**
 * @brief Joint模式的混合
 */
class QBlend {
public:
	QBlend(Q aMax, Q vMax, Q qMin, Q qMax);
	Interpolator<Q>::ptr query(State qStart, State qEnd, double maxDuration);
	virtual ~QBlend();
private:
	/** @brief Q的大小 */
	int _size;

	/** @brief 记录的最大加速度 */
	Q _aMax;

	/** @brief 记录的最大速度 */
	Q _vMax;

	Q _qMin;
	Q _qMax;
	/**> 至少要达到速度, 加速度限制的百分比 */
	const double _kMin = 0.9;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* QBLEND_H_ */
