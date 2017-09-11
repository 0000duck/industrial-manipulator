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

using robot::math::Q;
using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

class LinePlanner {
public:
	LinePlanner();
	Interpolator<Q>* query(Q qStart, Q qEnd);
	virtual ~LinePlanner();
private:
	double _vMaxLine;
	double _aMaxLine;
	double _hLine;
	double _vMaxAngle;
	double _aMaxAngle;
	double _hAngle;
	robot::ik::IKSolver* _ikSolver;
	Q _qMin;
	Q _qMax;
	Q _dqLim;
	Q _ddqLim;
    /**
     * @brief 机器人的模型
     */
    robot::model::SerialLink* _serialLink;
    SmoothMotionPlanner _smPlanner;
    std::vector<LinearCompositeInterpolator<double>*> _lcInterpolatorList;
    std::vector<LinearInterpolator<Vector3D<double> >*> _lvInterpolatorList;
    std::vector<LinearInterpolator<Rotation3D<double> >*> _lrInterpolatorList;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* LINEPLANNER_H_ */
