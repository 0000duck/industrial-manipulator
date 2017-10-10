/*
 * Trajectory.h
 *
 *  Created on: Sep 29, 2017
 *      Author: a1994846931931
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

# include "ConvertedInterpolator.h"
# include "SequenceInterpolator.h"
# include <memory>

using namespace robot::trajectory;

namespace robot {
namespace trajectory {

class Trajectory : public ikInterpolator{
public:
	struct qVelAcc{
		vector<Q> dq;
		vector<Q> ddq;
	};
public:
	using ptr = std::shared_ptr<Trajectory>;
	Trajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
			std::shared_ptr<robot::ik::IKSolver> iksolver,
			robot::model::Config config);

	qVelAcc sampleVelAcc(const int count, double precision=0.00001);

	vector<Q> sampleVel(const int count, double precision=0.00001);

	/**
	 * @brief 采样分析ikInterpolator指示路径上的关节速度和关节加速度上下限
	 * @param count [in] 采样的点数
	 * @param precision [in] 速度, 加速度数值计算的精度(dt)
	 * @return {{dqMin, dqMax}, {ddqMin, ddqMax}}
	 */
	std::pair<std::pair<Q, Q>, std::pair<Q, Q> > getMinMax(const int count = 100, double precision=0.00001);

	/**
	 * @brief 采样分析ikInterpolator指示路径上的关节速度和关节加速度的绝对值上限
	 * @param count [in] 采样的点数
	 * @param precision [in] 速度, 加速度数值计算的精度(dt)
	 * @return {dqMaxAbs, ddqMaxAbs}
	 */
	std::pair<Q, Q> getMaxAbs(const int count = 100, double precision=0.00001);

	/**
	 * @brief dqMax约束下采样沿路径的最大速度
	 * @param count [in] 采样的点数
	 * @param dqMax [in] 各关节的限制最大速度
	 * @param precision [in] 速度, 加速度数值计算的精度(dt)
	 * @return 在dqMax的约束下, 在采样点上沿着路径允许的最大速度
	 */
	vector<double> sampleMaxSpeed(const int count, Q dqMax, double precision=0.00001);

	/**
	 * @brief dqMax约束下获取沿路径的最大速度
	 * @param count [in] 采样的点数
	 * @param dqMax [in] 各关节的限制最大速度
	 * @param v [in] 限制沿路径的最大速度
	 * @param precision [in] 速度, 加速度数值计算的精度(dt)
	 * @return 在dqMax的约束下, 在整条路径上沿着路径允许的最大速度
	 */
	double getMaxSpeed(const int count, Q dqMax, double v, double precision=0.00001);

	/**
	 * @brief dqMax约束下获取沿路径的最大速度
	 * @param count [in] 采样的点数
	 * @param dqMax [in] 各关节的限制最大速度
	 * @param ddqMax [in] 各关节的限制最大加速度
	 * @param v [in] 限制沿路径的最大速度
	 * @param precision [in] 速度, 加速度数值计算的精度(dt)
	 * @return 在dqMax和ddqMax的约束下, 在整条路径上沿着路径允许的最大速度
	 */
	double getMaxSpeed(const int count, Q dqMax, Q ddqMax, double v, double precision=0.00001);
	virtual ~Trajectory(){}
protected:
	SequenceInterpolator<double>::ptr _lt;
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* TRAJECTORY_H_ */
