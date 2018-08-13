/**
 * @brief QtoQPlanner.h
 * @date Oct 27, 2017
 * @author a1994846931931
 */

#ifndef QTOQPLANNER_H_
#define QTOQPLANNER_H_

# include "Planner.h"
# include "../ik/IKSolver.h"
# include "../trajectory/ConvertedInterpolator.h"
# include "../trajectory/SequenceInterpolator.h"

using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief MoveJ点到点平滑规划(Planner派生的标准规划器)
 */
class QtoQPlanner {
public:
	using ptr = std::shared_ptr<Planner>;

	/**
	 * @brief 构造函数
	 * @param dqLim [in] 关节速度限制
	 * @param ddqLim [in] 关节加速度限制
	 * @param start [in] 开始位置
	 * @param qEnd [in] 结束位置
	 */
	QtoQPlanner(Q dqLim, Q ddqLim,
			Q start, Q qEnd);

	/**
	 * @brief 规划路径
	 * @return 规划的路径
	 */
	Interpolator<Q>::ptr query();

	/**
	 * @brief Planner操作 - 执行规划
	 */
	void doQuery();

	/**
	 * @brief Planner操作 - 执行暂停规划
	 * @param t [in] 从插补器时间t开始暂停
	 * @param stopIpr [in] 规划的沿路径暂停轨迹(若可暂停).
	 * @retval true 成功规划暂停路径
	 * @retval false 无法规划暂停路径
	 *
	 * - 当规划器路径尚未进行规划就执行暂停命令时会抛出一个错误.
	 * - 当由于剩余路径不足而导致无法进行规划时, 会打印出错误并return false.
	 */
	bool stop(double t, Interpolator<Q>::ptr& stopIpr);

	/**
	 * @brief Planner操作 - 执行恢复命令
	 *
	 * 从记录的暂停点和目标点之间进行轨迹规划.
	 */
	void resume(); //qStart为恢复点, 可以改为自动获取, 或留以作为位置误差判断

	/**
	 * @brief Planner操作 - 判断内部路径是否存在
	 * @return 内部路径是否存在
	 */
	bool isTrajectoryExist() const;

	/**
	 * @brief Planner操作 - 获取路径插补器
	 * @return 路径插补器
	 */
	Interpolator<Q>::ptr getQTrajectory() const;

	virtual ~QtoQPlanner(){}
private:
	/**> 关节速度限制 */
	Q _dqLim;

	/**> 关节加速度线坠 */
	Q _ddqLim;

	/**> 开始位置或暂停位置 */
	Q _qStop;

	/**> 结束位置(目标位置 */
	Q _qEnd;

	/**> 关节个数 */
	const double _size;

	/**> 沿虚拟映射路径的速度加速度和加加速度 */
	double _v, _a, _h;

	/**> 各个关节的旋转角度和映射长度L的比例 */
	std::vector<double> _k;

	/**> 各个关节以长度l为索引, 映射到自身角度变化的插补器 */
	std::vector<Interpolator<double>::ptr> _ltoqIpr;

	/**> 映射长度l关于时间的速度规划 */
	SequenceInterpolator<double>::ptr _lt;

	/**> 规划的路径 */
	Interpolator<Q>::ptr _qIpr;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* QTOQPLANNER_H_ */
