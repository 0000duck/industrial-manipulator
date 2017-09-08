/**
 * @brief PointToPointPlanner类
 * @date Sep 5, 2017
 * @author a1994846931931
 */

#ifndef POINTTOPOINTPLANNER_H_
#define POINTTOPOINTPLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"

using robot::math::Q;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/** @addtogroup pathplanner
 * @{
 */
class PointToPointPlanner {
public:
	PointToPointPlanner();
	Interpolator<Q>* querry(Q qStart, Q qDistance);

	virtual ~PointToPointPlanner();
};

/** @} */
} /* namespace pathplanner */
} /* namespace robot */

#endif /* POINTTOPOINTPLANNER_H_ */
