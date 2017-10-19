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
# include "../common/common.h"

using robot::pathplanner::Planner;

namespace robot {
namespace simulation {

typedef enum{
	stackEmpty=0, //栈空
	stackWait, //堆栈就绪, 等待开始指令状态
	stackNormal, //堆栈打开, 运行正常路径状态
	stackPause, //堆栈关闭, 运行暂停轨迹状态(阻塞状态, 不可中途重新启动)
	stackStop //堆栈关闭, 已完成暂停任务, 等待恢复指令
} stackStatus;

class MotionStack {
public:
	MotionStack(Q& initialQ);
	/**
	 * @brief 向堆栈中添加路径
	 * @param planner [in] 添加的规划器指针, 规划器应事先规划好一条路径
	 * @return 结果信息
	 * @retval 0 添加成功
	 * @retval 1 堆满
	 * @retval 2 添加路径的初始位置和上一条路径的结束位置不衔接
	 * @retval 3 添加的规划器中不包含路径
	 * @retval 4 添加的路径始末速度加速度不为0
	 */
	int addPlanner(Planner::ptr planner);

	/**
	 * @brief 启动堆
	 *
	 * 若成功启动记录下当前时间作为插补器开始的时间
	 * @return 结果信息
	 * @retval 0 启动成功
	 * @retval 1 栈空
	 * @retval 2 非正常等待状态
	 */
	int start();

	/**
	 * @brief 获取机器人插补状态
	 * @param t [in] 由getUTime()获取的绝对系统时间, 微秒
	 * @param state [out] 机器人状态
	 * @return 结果信息
	 * @retval 0 获取成功
	 * @retval 1 路径已到终点(此时发送路径末端状态. 正常发送情况下移除运动堆栈尾部, 暂停
	 * 路径情况下不移除. 最后切换运动堆栈状态)
	 * @retval 2 处于非发送状态
	 */
	int state(unsigned long long t, State &state);

	/**
	 * @brief 路径上暂停
	 * @retval 0 暂停成功
	 * @retval 1 无法暂停
	 * @retval 2 非正常发送状态
	 */
	int pause();

	/**
	 * @brief 从暂停停止状态恢复(阻塞型: 需要一定耗时来完成规划)
	 * @param current [in] 重新规划的开始点(传感器采集数据, 与停止点不能相差过大)
	 * @retval 0 成功恢复
	 * @retval 1 不是处于暂停停止阶段
	 * @warning 恢复后堆栈不会自动打开, 仍需调用start函数
	 */
	int resume(Q &current);

	/**
	 * @brief 清空栈
	 * @retval true 清空成功
	 * @retval false 运行途中无法清空
	 */
	bool clear();

	inline int getStatus()const {return _status;}

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

	int _status;
	/**> 记录开始时间 微秒 */
	unsigned long long int _recordTime;
	/**> 用于存放停止路径 */
	Interpolator<Q>::ptr _stopIpr;

	Q& _staticQ;

	int _size;
private:
	const Q _zero;
};

} /* namespace simulation */
} /* namespace robot */

#endif /* MOTIONSTACK_H_ */
