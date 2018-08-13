/**
 * @brief RotationPlanner.h
 * @date Oct 23, 2017
 * @author a1994846931931
 */

#ifndef ROTATIONPLANNER_H_
#define ROTATIONPLANNER_H_

# include "../trajectory/RotationInterpolator.h"
# include "../trajectory/LineTrajectory.h"
# include "Planner.h"

using robot::trajectory::RotationInterpolator;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief 纯旋转运动规划器
 *
 * 位置用FixedInterpolator, 姿态用RotationInterpolator实现
 */
class RotationPlanner : public Planner{
public:
	/**
	 * @brief 构造函数
	 * @param dqLim [in] 关节速度限制
	 * @param ddqLim [in] 关节加速度限制
	 * @param vMaxRot [in] 旋转运动速度限制
	 * @param aMaxRot [in] 旋转运动加速度限制
	 * @param hRot [in] 旋转运动加加速度限制
	 * @param ikSolver [in] 逆解器
	 * @param start [in] 开始位置
	 * @param n [in] 旋转轴
	 * @param theta [in] 旋转角度
	 *
	 * @note LinePlanner也可以实现纯旋转运动的规划, 但是它基于LinearInterpolator,
	 * 它会自动选择角度最小的旋转方向, 且旋转角度也自然的不能超过180°. 如果需要指定旋转
	 * 方向, 或者需要旋转角度大于180°的情况下, 就需要用此规划器显示的指定旋转方向和旋转角度.
	 */
	RotationPlanner(Q dqLim, Q ddqLim,
			double vMaxRot, double aMaxRot, double hRot,
			std::shared_ptr<robot::ik::IKSolver> ikSolver,
			Q start, Vector3D<double> n, double theta);

	/**
	 * @brief 规划路径
	 * @return 路径
	 *
	 * @note 没有给它实现专门的类, 而是用LineTrajectory来代替.
	 * 但有一点不同的是, 在直线运动中LineTrajectory使用直线段的长度作为索引, 提供l(t),
	 * dl(t)和ddl(t)获取沿直线运动的信息, 在纯旋转运动中, 由于没有位置上的移动, 因此改为由
	 * 旋转角度作为索引. l(t), dl(t)和ddl(t) (尽管从命名上是线段长度的意思)在这里返回的是
	 * 旋转的角度, 角速度和角加速度.
	 */
	LineTrajectory::ptr query();

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

	virtual ~RotationPlanner();
public:
	/**
	 * @brief 查找沿旋转方向正方向纯旋转运动可到达的最远点
	 * @param start [in] 开始位置
	 * @param n [in] 旋转方向
	 * @param ikSolver [in] 逆解器
	 * @param da [in] 采样步长(rad)
	 * @return 最远可到达的角度
	 * @note 为防止可能存在无限旋转的情况, 函数实现中限制了最远的距离为 2PI
	 */
	static double findReachableTheta(Q start, Vector3D<double> n, std::shared_ptr<robot::ik::IKSolver> ikSolver, double da=0.05);
private:
	/** @brief 关节最大速度 */
	Q _dqLim;

	/** @brief 关节最大加速度 */
	Q _ddqLim;

	/** @brief 末端预定最大角速度 */
	double _vMax;

	/** @brief 末端预定最大角加速度 */
	double _aMax;

	/** @brief 末端预定最大角加加速度 */
	double _h;

	/** @brief 逆解器 */
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

    /** @brief 机器人的模型 */
    robot::model::SerialLink::ptr _serialLink;

	/** @brief 关节个数 */
	int _size;

	Vector3D<double> _n;

	/**> 需要旋转的角度, 暂停后需要重置 */
	double _theta;

    /**> 记录的停止点, 用于恢复运动时检查启动点的位置正不正确 */
    Q _qStop;

    /**> 保存的config参数 */
    robot::model::Config _config;

    /**> 规划的路径 */
    LineTrajectory::ptr _lineTrajectory;

	/** @brief 采样精度 rad */
	const double _da = 0.5;

	/** @brief 最少采样点数 */
	const double _countMin = 8;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* ROTATIONPLANNER_H_ */
