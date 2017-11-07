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
 * @brief 直线规划器
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
	 * @param vMaxAngle [in] 直线路径最大角速度
	 * @param aMaxAngle [in] 直线路径最大角加速度
	 * @param hAngle [in] 直线路径角加加速度
	 * @param ikSolver [in] 逆解器
	 * @param serialLink [in] 机器人模型
	 */
	LinePlanner(Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine,
			std::shared_ptr<robot::ik::IKSolver> ikSolver,
			Q start, Q qEnd);

	/**
	 * @brief 询问路径(方法1)
	 * @param qStart [in] 起始位置
	 * @param qEnd [in] 终点位置
	 * @param speedRatio [in] 速度占最大速度的比例
	 * @param accRatio [in] 加速度占最大加速度的比例
	 * @return 直线路径的Q插补器
	 *
	 * 不考虑直径的旋转约束, 这样做的好处是可以把位姿以路径长度为索引. 方法2中直线
	 * 和旋转的速度规划是不统一的, 可能会造成"不同速度配置下, 路径不同"的结果;
	 */
	LineTrajectory::ptr query();

	/**
	 * @brief 询问路径(方法2)
	 * @param qStart [in] 起始位置
	 * @param qEnd [in] 终点位置
	 * @return 直线路径的Q插补器
	 * @deprecated
	 *
	 * 考虑路径的直线和旋转约束. 优点是可以考虑旋转约束. 但是在不同速度配置下, 会
	 * 出现不同的路径.
	 */
	LineTrajectory::ptr query2(const Q qStart, const Q qEnd) const;

	void doQuery();
	bool stop(double t, Interpolator<Q>::ptr& stopIpr);
	void resume(); //qStart为恢复点, 可以改为自动获取, 或留以作为位置误差判断
	bool isTrajectoryExist() const;
	Interpolator<Q>::ptr getQTrajectory() const;

	virtual ~LinePlanner();

public:
	/**
	 * @brief 纯直线平移查找可到达的最远距离
	 * @param start
	 * @param direction
	 * @param ikSolver
	 * @return
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

    robot::model::Config _config;

    LineTrajectory::ptr _lineTrajectory;

	/** @brief 采样精度 */
	const double _dl = 0.1;

	/** @brief 最少采样点数 */
	const double _countMin = 8;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* LINEPLANNER_H_ */
