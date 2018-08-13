/**
 * @brief MotionStack.h
 * @date Oct 17, 2017
 * @author a1994846931931
 */

#ifndef MOTIONSTACK_H_
#define MOTIONSTACK_H_

# include <queue>
# include "../pathplanner/Planner.h"
# include "../common/common.h"
# include <memory>
# include <mutex>

using robot::pathplanner::Planner;

namespace robot {
namespace simulation {

/** @addtogroup simulation
 * @brief 仿真类和运动堆栈等
 * @{
 */

/**> 堆栈状态 */
typedef enum{
	stackEmpty=0, //栈空
	stackWait, //堆栈就绪, 等待开始指令状态
	stackNormal, //堆栈打开, 运行正常路径状态
	stackPause, //堆栈关闭, 运行暂停轨迹状态(阻塞状态, 不可中途重新启动)
	stackStop //堆栈关闭, 已完成暂停任务, 等待恢复指令
} stackStatus;

/**
 * @brief 运动堆栈类
 *
 * 可以向运动堆栈添加已规划好路径的路径规划器(需要是由Planner派生的标砖运动规划器).
 * 使用start()函数可以启动运动堆栈, 运动堆栈记录时间戳并进入正常可获取状态. 此时调用state函数
 * 可以获取当前时间点的机器人规划状态. 时间应由"getUTime()"获取系统时间得到. 一条轨迹运动完成后运动堆栈会自动关闭
 * (即使还有其它轨迹存在). 此时, 再调用state函数会返回相应错误状态. 用户或状态机可以选择立即重新打开运动堆栈, 或者选择合适的时间
 * (例如等待机械臂稳定后)再打开. 使用pause函数可以询问一条暂停命令, 该函数是阻塞式的, 所以规划器中的pause函数应当足够快
 * (一般可以达到这个要求, 因为暂停路径很短, 不用花费很长时间去检查路径). 询问成功后可以继续用state函数获取, 此时获取的是
 * 沿路经暂停的轨迹. 完成暂停后, 可以用resume进行恢复路径的规划. 任何时候调用state函数应当都是可以返回一个State的, 所以
 * 处于暂停状态时应当记录暂停的关节位置(请在后续的编程中检查这一点).
 */
class MotionStack {
public:
	using ptr = std::shared_ptr<MotionStack>;

	MotionStack(Q& initialQ, std::mutex *mtx);
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
	 * @note id计数会清零
	 */
	bool clear();

	/**
	 * @brief 获取堆栈状态
	 * @return 状态值
	 */
	inline int getStatus()const {return _status;}

	/**> 获取锁 */
	inline std::mutex* getMutex(){return _mtx;}

	virtual ~MotionStack();
protected:
	struct motionData{
		int id;
		Planner::ptr planner;
		double duration;
	};

	/**
	 * @brief 运动队列, 记录添加到堆栈中的规划器指针, ID号和时长.
	 */
	std::queue<motionData> _motionQueue;
private:
	/**> 堆栈锁 */
	std::mutex *const _mtx; //未使用

	/**> 指向最高的ID号, 清空堆栈时清零 */
	int _id;

	/**> 最大栈数 */
	const int _maxDataCount = 100;

	/**> 堆栈状态 */
	int _status;

	/**> 记录开始时间 微秒 */
	unsigned long long int _recordTime;

	/**> 用于存放停止路径 */
	Interpolator<Q>::ptr _stopIpr;

	/**> 记录的非运行状态时的关节角度, 用于state返回 */
	Q& _staticQ;

	/**> 关节个数 */
	int _size;
private:
	const Q _zero;
};

/** @} */

} /* namespace simulation */
} /* namespace robot */

#endif /* MOTIONSTACK_H_ */
