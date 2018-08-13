/*
 * ExcessMotionPlanner.h
 *
 *  Created on: Sep 18, 2017
 *      Author: zrf
 */

#ifndef EXCESSMOTIONPLANNER_H_
#define EXCESSMOTIONPLANNER_H_

# include "../trajectory/Interpolator.h"
# include <vector>
# include "../trajectory/SequenceInterpolator.h"

namespace robot {
namespace pathplanner {


class ExcessMotionPlanner {
public:
	ExcessMotionPlanner();

	robot::trajectory::SequenceInterpolator<double>::ptr query(double s, double vMax, double aMax, double j, double ve, double vs,double start ) const;

	robot::trajectory::SequenceInterpolator<double>::ptr Motion7(double s, double vMax, double aMax, double j, double ve, double vs,double start) const;

	robot::trajectory::SequenceInterpolator<double>::ptr Motion6(double s, double vMax, double aMax, double j, double ve, double vs,double start) const;

	robot::trajectory::SequenceInterpolator<double>::ptr Motion22(double s, double vMax, double aMax, double j, double ve, double vs,double start) const;

	robot::trajectory::SequenceInterpolator<double>::ptr Motion32(double s, double vMax, double aMax, double j, double ve, double vs,double start) const;

	robot::trajectory::SequenceInterpolator<double>::ptr Motion23(double s, double vMax, double aMax, double j, double ve, double vs,double start) const;

	robot::trajectory::SequenceInterpolator<double>::ptr Motion302(double s, double vMax, double aMax, double j, double ve, double vs,double start) const;

	robot::trajectory::SequenceInterpolator<double>::ptr Motion203(double s, double vMax, double aMax, double j, double ve, double vs,double start) const;

	robot::trajectory::SequenceInterpolator<double>::ptr Motion202(double s, double vMax, double aMax, double j, double ve, double vs,double start) const;
	virtual ~ExcessMotionPlanner();
private:

	double st1(double j, double vMax, double vs, double aMax) const;

	double st2(double j, double vMax, double ve, double aMax) const;

	double ss1(double j, double vMax, double vs, double aMax) const;

	double ss2(double j, double vMax, double ve, double aMax) const;


};

}
}
#endif /* EXCESSMOTIONPLANNER_H_ */
