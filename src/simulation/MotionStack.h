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
	/**
	 * @brief 向堆栈中添加路径
	 * @param planner [in] 添加的规划器指针, 规划器应事先规划好一条路径
	 * @return 添加的结果
	 * @retval 0 添加成功
	 * @retval 1 堆满
	 * @retval 2 添加路径的初始位置和上一条路径的结束位置不衔接
	 * @retval 3 添加的规划器中不包含路径
	 */
	int addPlanner(Planner::ptr planner);
	virtual ~MotionStack();
protected:
	struct motionData{
		int id;
		Planner::ptr planner;
		double duration;
	};

	std::queue<motionData> _motionQueue;
private:
	/**> 指向最高的ID号, 清空堆栈时清零 */
	int _id;
	/**> 最大栈数 */
	const int _maxDataCount = 100;
	/**
	 * @brief 状态
	 *
	 *  - 0: 栈空
	 *  - 1: 栈不空, 等待状态
	 *  - 2: 栈不空, 正常发送状态
	 *  - 3: 栈不空, 运行暂停状态
	 */
	int _status;

};

} /* namespace simulation */
} /* namespace robot */

#endif /* MOTIONSTACK_H_ */
