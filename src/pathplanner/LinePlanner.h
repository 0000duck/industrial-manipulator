/**
 * @brief LinePlanner类
 * @date Sep 11, 2017
 * @author a1994846931931
 */

#ifndef LINEPLANNER_H_
#define LINEPLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"
# include "../ik/IKSolver.h"
# include <vector>
# include "../model/SerialLink.h"
# include "../model/Config.h"
# include "../trajectory/LineTrajectory.h"
# include "../trajectory/LinearInterpolator.h"
# include "Planner.h"
# include <memory>

using robot::math::Q;
using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

/** @addtogroup pathplanner
 * @{
 */

/**
 * @brief MoveL直线规划器(Planner派生的标准规划器)
 *
 * 给定两个点, 规划中间的路径, 返回Q插补器. 需要指明机器人各个关节的
 * 最大速度和最大加速度, 以及要规划路线的最大速度, 最大加速度, 以及加加速度.
 * - 如果按照路线设置的最大速度, 最大加速度, 以及加加速度规划出符合关节约束的路径, 则返回这条路径.
 * - 如果直线段有路径无法到达, 则抛出错误.
 * - 如果路径超出了关节的最大速度约束或者最大加速度约束, 则降低速度, 返回降速后的路径(插补器).
 * - 规划器会检查起始和结束位置的config参数, 若config参数不相同, 则抛出错误(直线规划器的首末config必须相同)
 */
class LinePlanner : public Planner {
public:
	using ptr = std::shared_ptr<LinePlanner>;

	/**
	 * @brief 构造函数
	 * @param dqLim [in] 关节最大速度
	 * @param ddqLim [in] 关节最大加速度
	 * @param vMaxLine [in] 直线路径最大速度
	 * @param aMaxLine [in] 直线路径最大加速度
	 * @param hLine [in] 直线路径加加速度
	 * @param ikSolver [in] 逆解器
	 * @param start [in] 开始位置
	 * @param qEnd [in] 结束位置
	 */
	LinePlanner(Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine,
			std::shared_ptr<robot::ik::IKSolver> ikSolver,
			Q start, Q qEnd);

	/**
	 * @brief Planner操作 - 询问路径
	 * @return 直线路径的Q插补器
	 *
	 * 不考虑直径的旋转约束, 这样做的好处是可以把位姿以路径长度为索引.
	 */
	LineTrajectory::ptr query();

	/**
	 * @brief Planner操作 - 询问路径
	 *
	 * 调用query()执行. 仅在内部保存路径, 而不进行返回.
	 */
	void doQuery();

	/**
	 * @brief Planner操作 - 执行暂停规划
	 * @param t [in] 从插补器时间t开始暂停
	 * @param stopIpr [in] 规划的沿直线暂停轨迹(若可暂停).
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

	virtual ~LinePlanner();

public:
	/**
	 * @brief 查找沿正方向纯直线平移可到达的最远点
	 * @param start [in] 开始点位置
	 * @param direction [in] 平移的方向
	 * @param ikSolver [in] 逆解器
	 * @param dl [in] 采样步长(m)
	 * @return 最远点
	 */
	static Q findReachableEnd(Q start, Vector3D<double> direction, std::shared_ptr<robot::ik::IKSolver> ikSolver, double dl=0.01);

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

    Q _qEnd;

    /**> 记录的停止点, 用于恢复运动时检查启动点的位置正不正确 */
    Q _qStop;

    /**> 机器人姿态 */
    robot::model::Config _config;

    /**> 规划器记录的直线轨迹 */
    LineTrajectory::ptr _lineTrajectory;

	/** @brief 采样精度 */
	const double _dl = 0.05; //设置过小会影响时间最优效率

	/** @brief 最少采样点数 */
	const double _countMin = 8;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* LINEPLANNER_H_ */
