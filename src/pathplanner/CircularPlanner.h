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
 * 圆弧路径规划器
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
	 * @param vMaxAngle [in] 直线路径最大角速度
	 * @param aMaxAngle [in] 直线路径最大角加速度
	 * @param hAngle [in] 直线路径角加加速度
	 * @param ikSolver [in] 逆解器
	 * @param serialLink [in] 机器人模型
	 */
	CircularPlanner(Q dqLim, Q ddqLim,
			double vMaxLine, double aMaxLine, double hLine,
			std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink::ptr serialLink,
			const Q qIntermediate, const Q qEnd);

	/**
	 * @brief 询问路径
	 * @param qStart [in] 圆弧开始点的关节数值
	 * @param qIntermediate [in] 圆弧中间点的关节数值
	 * @param qEnd [in] 圆弧结束点的关节数值
	 * @param speedRatio [in] 速度占最大速度的比例
	 * @param accRatio [in] 加速度占最大加速度的比例
	 * @return
	 */
	CircularTrajectory::ptr query(const Q qStart);

	bool stop(double t, Interpolator<Q>::ptr& stopIpr);
	void resume(const Q qStart); //qStart为恢复点, 可以改为自动获取, 或留以作为位置误差判断
	bool isTrajectoryExist() const;
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

    /** @brief 机器人的模型 */
    robot::model::SerialLink::ptr _serialLink;

    Q _qIntermediate;
    Q _qEnd;
    Q _qStop;

    robot::model::Config _config;

    CircularTrajectory::ptr _circularTrajectory;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* CIRCULARPLANNER_H_ */
