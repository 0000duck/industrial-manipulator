/**
 * @brief PointToPointPlanner类
 * @date Sep 5, 2017
 * @author a1994846931931
 */

#ifndef POINTTOPOINTPLANNER_H_
#define POINTTOPOINTPLANNER_H_

namespace robot {
namespace pathplanner {

/** @addtogroup pathplanner
 * @brief 路径规划器.
 * @{
 */
class PointToPointPlanner {
public:
	PointToPointPlanner();
	virtual ~PointToPointPlanner();
};

/** @} */
} /* namespace pathplanner */
} /* namespace robot */

#endif /* POINTTOPOINTPLANNER_H_ */
