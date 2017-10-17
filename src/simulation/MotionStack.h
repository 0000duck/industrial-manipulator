/*
 * MotionStack.h
 *
 *  Created on: Oct 17, 2017
 *      Author: a1994846931931
 */

#ifndef MOTIONSTACK_H_
#define MOTIONSTACK_H_

# include <queue>
# include "../pathplanner/Planner.h"

using robot::pathplanner::Planner;

namespace robot {
namespace simulation {

class MotionStack {
public:
	MotionStack();
	bool addPlanner(Planner::ptr);
	virtual ~MotionStack();
protected:
	struct {
		int id;
		Planner::ptr planner;
		double length;
	} motionData;

	std::queue<motionData> _motionQueue;

	/**> 指向最高的ID号, 清空堆栈时清零 */
	int _id;
};

} /* namespace simulation */
} /* namespace robot */

#endif /* MOTIONSTACK_H_ */
