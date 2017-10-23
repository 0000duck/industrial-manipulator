/*
 * TaskStack.h
 *
 *  Created on: Oct 20, 2017
 *      Author: a1994846931931
 */

#ifndef TASKSTACK_H_
#define TASKSTACK_H_

# include "MotionStack.h"
# include "../ik/IKSolver.h"
# include <queue>
# include <memory>
# include <mutex>

using robot::ik::IKSolver;

namespace robot {
namespace simulation {

using std::vector;

class TaskStack {
public:
	TaskStack(MotionStack::ptr motionStack, Q start, Q dqLim, Q ddqLim, double vMax, double aMax, double h, std::shared_ptr<IKSolver> solver);
	void reset(Q start);

	/**
	 * @brief 设置任务堆栈模式
	 * @param mode [in]
	 *  - 0 : (默认). 阻塞模式. 此时调用添加任务的函数时, 会阻塞调用者. 函数会一直分析路径, 直到分析完毕并
	 *  添加到运动堆栈后才会返回. 如果出现错误, 则返回false.
	 *  - 1 : 非阻塞模式. 添加任务的时候不会阻塞, 而是快速返回. 此时不会检查query以及添加到堆栈中的错误, 但能检查初级的
	 *  错误, 例如config, 参数错误等. ????
	 */
	void setMode(int mode);
	bool addLine(Q end, double vRatio, double aRatio);
	bool addCircle(Q intermediate, Q end, double vRatio, double aRatio);
	bool addMLAB(vector<Q> qPath, vector<double> arcRatio, vector<double> velocity, vector<double> acceleration, vector<double> jerk);
	virtual ~TaskStack();
private:
	void doQuery();
private:
	MotionStack::ptr _motionStack;

	std::mutex *const _msmtx;

	/** > _task 访问锁 */
	std::mutex _taskMutex;

	/**> doQuery函数锁 */
	std::mutex _queryMutex;

	Q _start;

	unsigned int _mode;

	std::queue<Planner::ptr> _task;

	const Q _dqLim;

	const Q _ddqLim;

	const double _velocity;

	const double _acceleration;

	const double _jerk;

	std::shared_ptr<IKSolver> _solver;

	bool _error;
};

} /* namespace simulation */
} /* namespace robot */

#endif /* TASKSTACK_H_ */
