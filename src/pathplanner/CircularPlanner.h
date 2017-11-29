/**
 * @brief CircularPlanner
 * @date Sep 18, 2017
 * @author a1994846931931
 */

#ifndef CIRCULARPLANNER_H_
#define CIRCULARPLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"
# include "../ik/IKSolver.h"
# include "SmoothMotionPlanner.h"
# include <vector>
# include "../model/SerialLink.h"
# include "../trajectory/CompositeInterpolator.h"
# include "../trajectory/LinearInterpolator.h"
# include "../trajectory/CircularTrajectory.h"
# include "Planner.h"
# include <memory>

using robot::math::Q;
using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * 圆弧路径规划器(Planner派生的标准规划器)
 */
class CircularPlanner : public Planner{
public:
	using ptr = std::shared_ptr<CircularPlanner>;

	/**
	 * @brief 构造函数
	 * @param dqLim [in] 关节最大速度
	 * @param ddqLim [in] 关节最大加速度
	 * @param vMaxLine [in] 直线路径最大速度
	 * @param aMaxLine [in] 直线路径最大加速度
	 * @param hLine [in] 直线路径加加速度
	 * @param ikSolver [in] 逆解器
	 * @param qStart [in] 开始位置
	 * @param qIntermediate [in] 中间位置
	 * @param qEnd [in] 终点位置
	 */
	CircularPlanner(Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine,
			std::shared_ptr<robot::ik::IKSolver> ikSolver,
			const Q qStart, const Q qIntermediate, const Q qEnd);

	/**
	 * @brief 询问路径
	 * @return 圆弧插补器
	 */
	CircularTrajectory::ptr query();

	/**
	 * @brief Planner操作 - 询问路径
	 *
	 * 调用query()执行. 仅在内部保存路径, 而不进行返回.
	 */
	void doQuery();

	/**
	 * @brief Planner操作 - 执行暂停规划
	 * @param t [in] 从插补器时间t开始暂停
	 * @param stopIpr [in] 规划的沿圆弧暂停轨迹(若可暂停).
	 * @retval true 成功规划暂停路径
	 * @retval false 无法规划暂停路径
	 *
	 * 暂停函数将计算新的中间点(若有必要)并记录, 以便进行路径的恢复.
	 * - 当规划器路径尚未进行规划就执行暂停命令时会抛出一个错误.
	 * - 当由于剩余路径不足而导致无法进行规划时, 会打印出错误并return false.
	 */
	bool stop(double t, Interpolator<Q>::ptr& stopIpr);

	/**
	 * @brief Planner操作 - 执行恢复命令
	 *
	 * 从记录的暂停点, 中间点和目标点之间进行轨迹规划.
	 */
	void resume();

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

	virtual ~CircularPlanner(){}
private:
	/** @brief 末端预定最大直线速度 */
	double _vMax;

	/** @brief 末端预定最大直线加速度 */
	double _aMax;

	/** @brief 末端预定最大直线加加速度 */
	double _h;

	/** @brief 逆解器 */
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

    /** @brief 机器人的模型 */
    robot::model::SerialLink::ptr _serialLink;

	/** @brief 关节下限 */
	Q _qMin;

	/** @brief 关节上限 */
	Q _qMax;

	/** @brief 关节最大速度 */
	Q _dqLim;

	/** @brief 关节最大加速度 */
	Q _ddqLim;

	/** @brief 关节个数 */
	int _size;

	/** @brief 起点位置(或者暂停时暂停的位置 */
    Q _qStop;

    /** @brief 中间点位置 */
    Q _qIntermediate;

    /** @brief 末端点位置 */
    Q _qEnd;

    /** @brief 整条路径上的config */
    robot::model::Config _config;

    /** @brief 圆弧插补器 */
    CircularTrajectory::ptr _circularTrajectory;

	/** @brief 采样精度 */
	const double _dl = 0.1;

	/** @brief 最少采样点数 */
	const double _countMin = 8;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* CIRCULARPLANNER_H_ */
