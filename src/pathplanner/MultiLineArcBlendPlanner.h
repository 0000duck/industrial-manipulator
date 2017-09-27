/*
 * MultiLineArcBlendPlanner.h
 *
 *  Created on: Sep 26, 2017
 *      Author: a1994846931931
 */

#ifndef MULTILINEARCBLENDPLANNER_H_
#define MULTILINEARCBLENDPLANNER_H_

# include "../trajectory/MLABTrajectory.h"
# include "../math/Q.h"
# include "../model/SerialLink.h"
# include "../math/HTransform3D.h"
# include "../trajectory/CircularInterpolator.h"
# include "../trajectory/LinearInterpolator.h"

using namespace robot::trajectory;
using std::vector;
using robot::model::SerialLink;
using robot::math::HTransform3D;

namespace robot {
namespace pathplanner {

class MultiLineArcBlendPlanner {
public:
	MultiLineArcBlendPlanner();
	MLABTrajectory::ptr query(const vector<Q>& path, const vector<double> arcRatio);
	virtual ~MultiLineArcBlendPlanner();
private:
private:
	const SerialLink* _robot;
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* MULTILINEARCBLENDPLANNER_H_ */
