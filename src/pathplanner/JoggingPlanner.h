/*
 * JoggingPlanner.h
 *
 *  Created on: Oct 19, 2017
 *      Author: a1994846931931
 */

#ifndef JOGGINGPLANNER_H_
#define JOGGINGPLANNER_H_

# include "Planner.h"
# include "../ik/IKSolver.h"

using std::vector;

namespace robot {
namespace pathplanner {

class JoggingPlanner {
public:
	JoggingPlanner(std::shared_ptr<robot::ik::IKSolver> ikSolver, vector<double> constraints, Q dqLim, Q ddqLim);

	Planner::ptr jogX(Q current, Q &farEnd, Rotation3D<double> rot=Rotation3D<double>());

	Planner::ptr jogY(Q current);

	Planner::ptr jogZ(Q current);

	Planner::ptr jogRX(Q current);

	Planner::ptr jogRY(Q current);

	Planner::ptr jogRZ(Q current);

	virtual ~JoggingPlanner();
private:
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

	double _vLine;

	double _aLine;

	double _jLine;

	double _vAngle;

	double _aAngle;

	double _jAngle;

	Q _dqLim;

	Q _ddqLim;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* JOGGINGPLANNER_H_ */
