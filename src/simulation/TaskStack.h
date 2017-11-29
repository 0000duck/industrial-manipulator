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
using std::vector;

namespace robot {
namespace simulation {

/** @addtogroup simulation
 * @{
 */

/**
 * @brief 任务堆栈, 使用任务堆栈方便地给运动堆栈添加轨迹.
 *
 * 需要指定一个初始的关节位置. 之后添加的路径不需要指定开始位置, 开始位置由上一条路径的末尾指定.
 * @warning 非阻塞模式可行性有待考虑.
 * @todo
 *  - 添加示教规划器的添加函数
 *  - 添加QtoQ实现函数
 */
class TaskStack {
public:
	/**
	 * @brief 构造函数
	 * @param motionStack [in] 绑定的运动堆栈(添加路径到该堆栈)
	 * @param start [in] 机器人初始状态
	 * @param dqLim [in] 关节速度限制
	 * @param ddqLim [in] 关节加速度限制
	 * @param vMax [in] 最大速度
	 * @param aMax [in] 最大加速度
	 * @param h [in] 加加速度
	 * @param solver [in] 逆解器
	 */
	TaskStack(MotionStack::ptr motionStack, Q start, Q dqLim, Q ddqLim, double vMax, double aMax, double h, std::shared_ptr<IKSolver> solver);

	/** @brief 重设开始位置和_error变量 */
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

	/**
	 * @brief 添加直线段MoveL
	 * @param end [in] 末端位置
	 * @param vRatio [in] 速度比例
	 * @param aRatio [in[ 加速度比例
	 * @return 是否成功添加
	 */
	bool addLine(Q end, double vRatio, double aRatio);

	/**
	 * @brief 添加圆弧段MoveC
	 * @param intermediate [in] 中间点
	 * @param end [in] 末端点
	 * @param vRatio [in] 速度比例
	 * @param aRatio [in] 加速度比例
	 * @return 是否成功添加
	 */
	bool addCircle(Q intermediate, Q end, double vRatio, double aRatio);

	/**
	 * @brief 添加圆弧插补的连续直线运动MoveL with blend
	 * @param qPath [in] 路径点
	 * @param arcRatio [in] 圆弧比例(混合区占整条直线的比例, 0.1~0.5)
	 * @param velocity [in] 各段的速度
	 * @param acceleration [in] 各段的加速度
	 * @param jerk [in] 各段的加加速度
	 * @return 是否成功添加
	 */
	bool addMLAB(vector<Q> qPath, vector<double> arcRatio, vector<double> velocity, vector<double> acceleration, vector<double> jerk);

	virtual ~TaskStack();
private:
	/**> 后台规划函数, 当模式为非阻塞时调用 */
	void doQuery();
private:
	/**> 保存的运动堆栈指针 */
	MotionStack::ptr _motionStack;

	/**> 运动堆栈锁指针 */
	std::mutex *const _msmtx;

	/** > _task 访问锁 */
	std::mutex _taskMutex;

	/**> doQuery函数锁 */
	std::mutex _queryMutex;

	/**> 下条规划指令的开始位置 */
	Q _start;

	/**> 模式, 指定添加路径为阻塞式还是非阻塞时 */
	unsigned int _mode;

	/**> 任务表, 非阻塞式后台规划函数从中获取规划器并执行规划 */
	std::queue<Planner::ptr> _task;

	/**> 关节速度限制 */
	const Q _dqLim;

	/**> 关节加速度限制 */
	const Q _ddqLim;

	/**> 速度限制 */
	const double _velocity;

	/**> 加速度限制 */
	const double _acceleration;

	/**> 加加速度限制 */
	const double _jerk;

	/**> 逆解器 */
	std::shared_ptr<IKSolver> _solver;

	/**> 当任务堆栈的某条路径无法规划时, 其后所有的路径都不能规划.
	 * 当error为true时, 无法再添加路径, 除非使用reset函数重置
	 */
	bool _error;
};

/** @} */

} /* namespace simulation */
} /* namespace robot */

#endif /* TASKSTACK_H_ */
