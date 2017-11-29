/**
 * @brief JoggingPlanner.h
 * @date Oct 19, 2017
 * @author a1994846931931
 */

#ifndef JOGGINGPLANNER_H_
#define JOGGINGPLANNER_H_

# include "Planner.h"
# include "../ik/IKSolver.h"

using std::vector;

namespace robot {
namespace pathplanner {

/**
 * @addtogroup pathplanner
 * @{
 */

/**
 * @brief 示教规划器, 可以实现x, y, z, rx, ry, rz的示教.
 *
 * 自动规划规划出当前点到可到达的最远点之间的路径.
 */
class JoggingPlanner {
public:
	/**
	 * @brief 构造函数
	 * @param ikSolver [in] 逆解器
	 * @param constraints [in] 速度加速度限制. 需要有六个数, 按照顺序分别是直线
	 * 速度, 直线加速度, 直线加加速度, 旋转速度, 旋转加速度, 旋转加加速度
	 * @param dqLim [in] 关节的速度限制
	 * @param ddqLim [in] 关节的加速度限制
	 */
	JoggingPlanner(std::shared_ptr<robot::ik::IKSolver> ikSolver, vector<double> constraints, Q dqLim, Q ddqLim);

	/**
	 * #brief X方向的示教规划
	 * @param current [in] 当前的关节角度值
	 * @param farEnd [out] 规划的最远端点的关节角度值
	 * @param isPositive [in] 正方向还是负方向
	 * @param rot [in] X方向所参考坐标系相对于世界坐标的旋转矩阵
	 * @return 示教规划器
	 */
	Planner::ptr jogX(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot=Rotation3D<double>());

	Planner::ptr jogY(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot=Rotation3D<double>());

	Planner::ptr jogZ(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot=Rotation3D<double>());

	Planner::ptr jogRX(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot=Rotation3D<double>());

	Planner::ptr jogRY(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot=Rotation3D<double>());

	Planner::ptr jogRZ(Q current, Q &farEnd, bool isPositive, Rotation3D<double> rot=Rotation3D<double>());

	virtual ~JoggingPlanner();

private:
	/**
	 * @brief 规划纯直线运动
	 * @param current [in] 当前的关节角度值
	 * @param farEnd [out] 规划的最远端点的关节角度值
	 * @param direction [in] 运行方向
	 * @return 示教规划器
	 */
	Planner::ptr planLine(Q current, Q &farEnd, Vector3D<double> direction);

	/**
	 * @brief 规划纯旋转运动
	 * @param current [in] 当前的关节角度值
	 * @param farEnd [out] 规划的最远端点的关节角度值
	 * @param direction [in] 旋转轴方向
	 * @return 示教规划器
	 */
	Planner::ptr planRotation(Q current, Q &farEnd, Vector3D<double> direction);
private:
	/**> 逆解器 */
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;

	double _vLine; //velocity

	double _aLine; //acceleration

	double _jLine; //jerk

	double _vAngle;

	double _aAngle;

	double _jAngle;

	Q _dqLim;

	Q _ddqLim;
};

/** @} */

} /* namespace pathplanner */
} /* namespace robot */

#endif /* JOGGINGPLANNER_H_ */
