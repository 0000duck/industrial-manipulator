/**
 * @brief Planner.h
 * @date Oct 17, 2017
 * @author a1994846931931
 */

#ifndef PLANNER_H_
#define PLANNER_H_

# include "../trajectory/Interpolator.h"
# include "../math/Q.h"
# include "../kinematics/State.h"
# include <memory>

using robot::trajectory::Interpolator;
using robot::math::Q;
using robot::kinematic::State;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief 规划器基类
 */
class Planner {
public:
	using ptr = std::shared_ptr<Planner>;

	Planner();

	/**
	 * @brief 执行规划
	 */
	virtual void doQuery() = 0;

	/**
	 * @brief 执行暂停
	 * @param t [in] 开始暂停的时刻(插补器时间).
	 * @param stopIpr [out] 规划的从t时刻开始的, 沿路经暂停的轨迹.
	 * @retval true 成功规划暂停路径.
	 * @retval false 无法规划暂停路径, 例如路径剩余距离太短.
	 */
	virtual bool stop(double t, Interpolator<Q>::ptr& stopIpr) = 0;

	/**
	 * @brief 从记录的暂停位置沿路经重新进行规划
	 */
	virtual void resume() = 0;

	/**
	 * @brief 判断规划器路径是否存在
	 * @return 规划器路径是否存在
	 */
	virtual bool isTrajectoryExist() const = 0;

	/**
	 * @brief 获取规划器的规划的路径
	 *
	 * 如果路径不存在, 可能会返回一个空指针.
	 * @return 规划器存储的路径
	 */
	virtual Interpolator<Q>::ptr getQTrajectory() const = 0;

	virtual ~Planner();
protected:
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* PLANNER_H_ */
