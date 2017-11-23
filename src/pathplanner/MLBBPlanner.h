/*
 * MLBBPlanner.h
 *
 *  Created on: Nov 23, 2017
 *      Author: a1994846931931
 */

#ifndef MLBBPLANNER_H_
#define MLBBPLANNER_H_

namespace robot {
namespace pathplanner {

/**
 * @brief 贝塞尔曲线混合的连续路径规划器
 */
class MLBBPlanner {
public:
	MLBBPlanner();
	virtual ~MLBBPlanner();
};

} /* namespace pathplanner */
} /* namespace robot */

#endif /* MLBBPLANNER_H_ */
