/*
 * LinePlanner.h
 *
 *  Created on: Sep 11, 2017
 *      Author: a1994846931931
 */

#ifndef LINEPLANNER_H_
#define LINEPLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"
# include "../ik/IKSolver.h"
# include "SmoothMotionPlanner.h"
# include <vector>
# include "../model/SerialLink.h"
# include "../trajectory/CompositeInterpolator.h"
# include "../trajectory/LinearInterpolator.h"
# include <memory>

using robot::math::Q;
using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

class LinePlanner {
public:
	LinePlanner(Q qMin, Q qMax, Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine, double vMaxAngle, double aMaxAngle, double hAngle,
			std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink* serialLink);
	Interpolator<Q>::ptr query(Q qStart, Q qEnd);
	virtual ~LinePlanner();
private:
	double _vMaxLine;
	double _aMaxLine;
	double _hLine;
	double _vMaxAngle;
	double _aMaxAngle;
	double _hAngle;
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;
	Q _qMin;
	Q _qMax;
	Q _dqLim;
	Q _ddqLim;
    /**
     * @brief 机器人的模型
     */
    robot::model::SerialLink* _serialLink;
    SmoothMotionPlanner _smPlanner;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* LINEPLANNER_H_ */
