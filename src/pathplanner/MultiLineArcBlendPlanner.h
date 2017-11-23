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
# include "../model/Config.h"
# include "Planner.h"
# include <queue>

using namespace robot::trajectory;
using std::vector;
using robot::model::SerialLink;
using robot::math::HTransform3D;
using std::queue;
using robot::model::Config;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief 连续规划-多直线圆弧混合路径规划器
 */
class MultiLineArcBlendPlanner : public Planner{
public:
	using ptr = std::shared_ptr<MultiLineArcBlendPlanner>;

	/**
	 * @brief 构造函数
	 * @param dqLim [in] 关节速度限制
	 * @param ddqLim [in] 关节加速度限制
	 * @param ikSolver [in] 逆解器
	 * @param path [in] 连续路径点
	 * @param arcRatio [in] 圆弧开始处到原相交点的距离占两侧最短直线长度的比例
	 * @param velocity [in] 在各段直线上的期望速度
	 * @param acceleration [in] 在各段直线上的期望加速度
	 * @param jerk [in] 在各段直线上的期望加加速度
	 */
	MultiLineArcBlendPlanner(Q dqLim, Q ddqLim,
			std::shared_ptr<robot::ik::IKSolver> ikSolver,
			const vector<Q>& path, const vector<double>& arcRatio, vector<double>& velocity, vector<double>& acceleration, vector<double>& jerk);

	MLABTrajectory::ptr query();

	vector<SequenceInterpolator<double>::ptr> getLt(vector<Trajectory::ptr>& trajectoryIpr);

	void doQuery();

	bool stop(double t, Interpolator<Q>::ptr& stopIpr);

	void resume(); //qStart为恢复点, 可以改为自动获取, 或留以作为位置误差判断

	bool isTrajectoryExist() const;

	Interpolator<Q>::ptr getQTrajectory() const;

	virtual ~MultiLineArcBlendPlanner();
private:
private:
	/** @brief 关节个数 */
	int _size;

	/** @brief 逆解器 */
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

    /** @brief 机器人的模型 */
	SerialLink::ptr _serialLink;

	/** @brief 关节下限 */
	const Q _qMin;

	/** @brief 关节上限 */
	const Q _qMax;

	/** @brief 关节最大速度 */
	const Q _dqLim;

	/** @brief 关节最大加速度 */
	const Q _ddqLim;

	Q _qStop;

	vector<vector<HTransform3D<double> > > _task;

	vector<double> _velocity;

	vector<double> _acceleration;

	vector<double> _jerk;

	bool _startFromLine;

	Config _config;

	MLABTrajectory::ptr _mLABTrajectory;

	/** @brief 采样精度 */
	const double _dl = 0.1;

	/** @brief 最少采样点数 */
	const double _countMin = 8;
};

/**@}*/

} /* namespace pathplanner */
} /* namespace robot */

#endif /* MULTILINEARCBLENDPLANNER_H_ */
