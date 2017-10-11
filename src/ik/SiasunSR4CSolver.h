/**
 * @brief 用于新松4kg和6kg机器人的逆运动学解算
 * @date Aug 28, 2017
 * @author a1994846931931
 */

#ifndef SIASUNSR4CSOLVER_H_
#define SIASUNSR4CSOLVER_H_

# include <vector>
# include "../math/Q.h"
# include "../model/DHTable.h"
# include "../model/SerialLink.h"
# include "../model/Config.h"
# include "IKSolver.h"
# include "../common/printAdvance.h"

using namespace robot::common;

namespace robot {
namespace ik {

/** @addtogroup ik
 * @brief 用于机器人逆运动学求解.
 *
 * 包括的类有:
 * 1. PieperSolver: 可用于符合Pieper准则及其它一些条件的特定机器人的逆运动学求解器
 * 2. SiasunSR4CSolver: 用于新松SR4C机器人, 以及其它相似构造机器人(如新松6kg机器人)的逆运动学求解器
 * @{
 */

/**
 * @brief 用于新松4kg机器人的机器人逆解器
 *
 * 也可用于新松6kg机器人, 或者构造相同的其它机器人
 */
class SiasunSR4CSolver: public IKSolver {
public:
	/**
	 * @brief 构造新松4kg机器人的逆解器
	 * @param serialRobot [in] 机器人模型
	 */
	SiasunSR4CSolver(robot::model::SerialLink::ptr serialRobot);

	/**
	 * @brief 初始化
	 */
	void init();

	/**
	 * @brief 逆运动学求解
	 * @param baseTend [in] 基坐标系到末端执行器的变换矩阵
	 * @param config [in] 逆解的Config参数
	 * @return 按照config参数求解, 总是给出1个或0个解
	 */
    std::vector<Q> solve(const HTransform3D<>& baseTend, const model::Config& config) const;
    void solveTheta456(double theta1,
                       double theta2,
                       double theta3,
                       robot::math::HTransform3D<>& T06,
                       std::vector<robot::math::Q>& result,
                       const model::Config&) const;

    /**
     * @brief 判断机器人模型可否由该逆解器求解
     * @todo 尚未实现
     * @retval true 可以求解
     * @retval false 不可求解
     */
    bool isValid() const;

    /**
     * @brief 判断关节角能否满足config中的肩部约束
     * @param q [in] 机器人的关节角
     * @param config [in] Config参数
     * @retval true 满足约束
     * @retval false 不满足约束
     */
	bool isShoulderValid(const robot::math::Q& q, const model::Config& config) const;

    /**
     * @brief 判断关节角能否满足config中的肩部约束
     * @param r [in] 第三轴原点到第一轴的距离
     * @param config [in] Config参数
     * @retval true 满足约束
     * @retval false 不满足约束
     */
	bool isShoulderValid(const double r, const model::Config& config) const;

    /**
     * @brief 判断关节角能否满足config中的肘部约束
     * @param q [in] 机器人的关节角
     * @param config [in] Config参数
     * @retval true 满足约束
     * @retval false 不满足约束
     */
	bool isElbowValid(const robot::math::Q& q, const model::Config& config) const;

    /**
     * @brief 判断关节角能否满足config中的肘部约束
     * @param j3 [in] 第三关节角度
     * @param config [in] Config参数
     * @retval true 满足约束
     * @retval false 不满足约束
     */
	bool isElbowValid(const double j3, const model::Config& config) const;

    /**
     * @brief 判断关节角能否满足config中的腕部约束
     * @param q [in] 机器人的关节角
     * @param config [in] Config参数
     * @retval true 满足约束
     * @retval false 不满足约束
     */
	bool isWristValid(const robot::math::Q& q, const model::Config& config) const;

    /**
     * @brief 判断关节角能否满足config中的腕部约束
     * @param j5 [in] 第五关节角度
     * @param config [in] Config参数
     * @retval true 满足约束
     * @retval false 不满足约束
     */
	bool isWristValid(const double j5, const model::Config& config) const;

	/**
	 * @brief 分析关节角所满足的是哪一种Config参数
	 * @param q [in] 要分析的关节角度数组
	 * @return 只会明式地给出匹配的Config类型 <br>
	 * 对于肩部, 返回的是lefty或righty <br>
	 * 对于肘部, 返回的是epositive或enegtive <br>
	 * 对于腕部, 返回的是wpositive或wnegtive
	 */
	robot::model::Config getConfig(const robot::math::Q& q) const;
	virtual ~SiasunSR4CSolver();
private:
    double _alpha1, _a1, _calpha1, _salpha1, _d1;
    double _alpha2, _a2, _calpha2, _salpha2, _d2;
    double _alpha3, _a3, _calpha3, _salpha3, _d3;
    double _alpha4, _a4, _calpha4, _salpha4, _d4;
    double _alpha5, _a5, _calpha5, _salpha5, _d5;
    double _alpha6, _a6, _calpha6, _salpha6, _d6;
private:
    /**
     * @brief Modified DH 参数表
     */
	robot::model::DHTable _dHTable;

	/**
	 * @brief 从基座到0关节的变换矩阵
	 */
    robot::math::HTransform3D<> _0Tbase;

    /**
     * @brief 从末端执行器到6关节的变换矩阵
     */
    robot::math::HTransform3D<> _endTjoint6;

    /**
     * @brief 机器人的模型
     */
    robot::model::SerialLink::ptr _serialLink;
};

/** @} */
} /* namespace ik */
} /* namespace robot */

#endif /* SIASUNSR4CSOLVER_H_ */
