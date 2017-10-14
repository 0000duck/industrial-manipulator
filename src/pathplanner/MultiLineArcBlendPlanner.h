/**
 * @brief MultiLineArcBlendPlanner
 * @date Sep 26, 2017
 * @author a1994846931931
 */

#ifndef MULTILINEARCBLENDPLANNER_H_
#define MULTILINEARCBLENDPLANNER_H_

# include "../trajectory/MLABTrajectory.h"
# include "../math/Q.h"
# include "../model/SerialLink.h"
# include "../math/HTransform3D.h"
# include "../trajectory/CircularInterpolator.h"
# include "../trajectory/LinearInterpolator.h"

using namespace robot::trajectory;
using std::vector;
using robot::model::SerialLink;
using robot::math::HTransform3D;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief 连续规划-多直线圆弧混合路径规划器
 */
class MultiLineArcBlendPlanner {
public:
	/**
	 * @brief 构造函数
	 * @param dqLim [in] 关节最大速度
	 * @param ddqLim [in] 关节最大加速度
	 * @param ikSolver [in] 逆解器
	 * @param serialLink [in] 机器人模型
	 */
	MultiLineArcBlendPlanner(Q dqLim, Q ddqLim,
			std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink::ptr serialLink);

	/**
	 * @brief 询问路径
	 * @param path [in] 关键点列表
	 * @param arcRatio [in] 各个交界处圆弧起始点占整条线段的比值(0.1~0.5)
	 * @param velocity [in] 各个线段上的期望速度
	 * @param acceleration [in] 各个线段上的期望加速度
	 * @param jerk [in] 各个线段上的最大加加速度
	 * @return 直线圆弧连续轨迹插补器的指针
	 */
	MLABTrajectory::ptr query(const vector<Q>& path, const vector<double>& arcRatio, vector<double>& velocity, vector<double>& acceleration, vector<double>& jerk);
	vector<SequenceInterpolator<double>::ptr> getLt(double arcSize, double lineSize, vector<Trajectory::ptr>& trajectoryIpr, vector<double>& velocity, vector<double>& acceleration, vector<double>& jerk);
	virtual ~MultiLineArcBlendPlanner();
private:
private:
	/** @brief 关节个数 */
	int _size;

	/** @brief 逆解器 */
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

	/** @brief 关节下限 */
	const Q _qMin;

	/** @brief 关节上限 */
	const Q _qMax;

	/** @brief 关节最大速度 */
	const Q _dqLim;

	/** @brief 关节最大加速度 */
	const Q _ddqLim;

    /** @brief 机器人的模型 */
	SerialLink::ptr _serialLink;
};

/**@}*/

} /* namespace pathplanner */
} /* namespace robot */

#endif /* MULTILINEARCBLENDPLANNER_H_ */
