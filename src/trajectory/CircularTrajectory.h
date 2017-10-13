/**
 * @brief CircularTrajectory
 * @date Oct 12, 2017
 * @author a1994846931931
 */

#ifndef CIRCULARTRAJECTORY_H_
#define CIRCULARTRAJECTORY_H_

# include "ConvertedInterpolator.h"
# include "CompositeInterpolator.h"
# include "Trajectory.h"
# include <memory>

using namespace robot::trajectory;

namespace robot {
namespace trajectory {

/**
 * @addtogroup trajectory
 * @{
 */

/**
 * @brief 圆弧路径插补器
 */
class CircularTrajectory : public Interpolator<Q>{
public:
	using ptr = std::shared_ptr<CircularTrajectory>;
	/**
	 * @brief 构造函数
	 * @param origin [in] 位置和姿态插补器的pair(时间为索引)
	 * @param iksolver [in] 逆解器
	 * @param config [in] 路径的config参数
	 * @param lt [in] 路径速度规划的插补器
	 * @param trajectory [in] 路径(长度为索引)
	 */
	CircularTrajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
			std::shared_ptr<robot::ik::IKSolver> iksolver,
			robot::model::Config config,
			SequenceInterpolator<double>::ptr lt,
			Trajectory::ptr trajectory);

	Q x(double t) const;
	Q dx(double t) const;
	Q ddx(double t) const;

	/**
	 * @brief 获取时间索引t处的路径长度
	 * @param t [in] 时间索引
	 * @return t处的路径长度
	 */
	double l(double t) const;

	/**
	 * @brief 获取时间索引t处的路径速度
	 * @param t [in] 时间索引
	 * @return t处的路径速度
	 */
	double dl(double t) const;

	/**
	 * @brief 获取时间索引t处的路径加速度
	 * @param t [in] 时间索引
	 * @return t处的路径加速度
	 */
	double ddl(double t) const;
	double duration() const;
	virtual ~CircularTrajectory(){}
private:
	ikInterpolator::ptr _qIpr;
	Trajectory::ptr _trajectory;

	/** @brief 直线距离-时间插补器 */
	SequenceInterpolator<double>::ptr _lt;

	std::vector<std::pair<double, double> > _lengthPath;
	const int _pathSize;
};

/** @} */

} /* namespace trajectory */
} /* namespace robot */

#endif /* CIRCULARTRAJECTORY_H_ */
