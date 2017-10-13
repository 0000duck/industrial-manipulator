/**
 * @brief Trajectory
 * @date Sep 29, 2017
 * @author a1994846931931
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

# include "ConvertedInterpolator.h"
# include "SequenceInterpolator.h"
# include <memory>

using namespace robot::trajectory;

namespace robot {
namespace trajectory {

/**
 * @addtogroup trajectory
 * @{
 */

/**
 * @brief 路径类
 *
 * 要求缩索引为长度, 提供一些采样方法.
 */
class Trajectory : public ikInterpolator{
public:
	struct qVelAcc{
		vector<Q> dq;
		vector<Q> ddq;
	};
public:
	using ptr = std::shared_ptr<Trajectory>;

	/**
	 * @brief 构造函数
	 * @param origin [in] 位姿插补器, 位置和姿态插补器的pair(长度为索引)
	 * @param iksolver [in] 用于逆解的逆解器
	 * @param config [in] 用于逆解的位姿参数
	 */
	Trajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
			std::shared_ptr<robot::ik::IKSolver> iksolver,
			robot::model::Config config);

	/**
	 * @brief 沿路经关节速度加速度采样
	 * @param count [in] 采样的点数
	 * @param precision [in] 速度, 加速度数值计算的精度(dt)
	 * @return 由速度采样点和加速度采样点构造的数据
	 */
	qVelAcc sampleVelAcc(const int count, double precision=0.00001);

	/**
	 * @brief 沿路经关节速度采样
	 * @param count [in] 采样的点数
	 * @param precision [in] 速度, 加速度数值计算的精度(dt)
	 * @return 速度采样点列表
	 */
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
};

/** @} */

} /* namespace trajectory */
} /* namespace robot */

#endif /* TRAJECTORY_H_ */
