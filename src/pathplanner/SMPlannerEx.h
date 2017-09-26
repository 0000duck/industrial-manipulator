/*
 * SMPlannerEx.h
 *
 *  Created on: Sep 25, 2017
 *      Author: a1994846931931
 */

#ifndef SMPLANNEREX_H_
#define SMPLANNEREX_H_

# include "../trajectory/Interpolator.h"
# include <vector>
# include "../trajectory/SequenceInterpolator.h"

namespace robot {
namespace pathplanner {

class SMPlannerEx {
public:
	SMPlannerEx();

	robot::trajectory::SequenceInterpolator<double>::ptr query(double s, double h, double aMax, double v1, double v2) const;
	bool checkDitance(double s, double h, double aMax, double v1, double v2) const;
	virtual ~SMPlannerEx(){}
private:
	robot::trajectory::SequenceInterpolator<double>::ptr threeLineMotion(double s, double h, double aMax, double v1, double v2) const;
	robot::trajectory::SequenceInterpolator<double>::ptr fourLineMotion(double s, double h, double aMax, double v1, double v2) const;
	robot::trajectory::SequenceInterpolator<double>::ptr threeLineMotion(double s, double h, double aMax, double v1) const;
	robot::trajectory::SequenceInterpolator<double>::ptr fourLineMotion(double s, double h, double aMax, double v1) const;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* SMPLANNEREX_H_ */
