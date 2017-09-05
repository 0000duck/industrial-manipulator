/**
 * @brief Pieper法机器人逆运动学求解
 * @date Aug 17, 2017
 * @author a1994846931931
 */

#ifndef PIEPERSOLVER_H_
#define PIEPERSOLVER_H_

# include <vector>
# include "../math/Q.h"
# include "../model/DHTable.h"
# include "../model/SerialLink.h"

namespace robot {
namespace ik {

/** @addtogroup ik
 * @{
 */

/**
 * @brief Pieper逆解器
 *
 * @warning 适用于Pieper逆解器的机器人模型必须满足: <br>
 * 	- 1. 末端三轴交于一点 <br>
 * 	- 2. 对于Modifed DH参数, 机器人第二个关节的a=0或者sin(alpha)=0
 */
class PieperSolver {
public:
	/**
	 * @brief 构造Pieper逆解器
	 * @param serialRobot [in] 机器人模型
	 */
	PieperSolver(robot::model::SerialLink& serialRobot);

	/**
	 * @brief 初始化函数, 当模型改变时, 可以重新计算参数, 一般不需要调用
	 */
    void init();

    /**
     * @brief Pieper法机器人逆运动学求解
     * @param baseTend 从基坐标系到执行末端的变换矩阵
     * @return 若有多个解, 则都返回
     */
    std::vector<Q> solve(const HTransform3D<>& baseTend) const;
    void solveTheta456(double theta1,
                       double theta2,
                       double theta3,
                       robot::math::HTransform3D<>& T06,
                       std::vector<robot::math::Q>& result) const;


    /**
     * @brief Solves the case with a1 = 0 (Case 1 in Craig)
     */
    std::vector<double> solveTheta3Case1(double z) const;

    /**
     * @brief Solves the case with a1 != 0 and sin(alpha1) = 0 (Case 2 in Craig)
     */
    std::vector<double> solveTheta3Case2(double r) const;

    /**
     * @brief Solves the general case with a1 != 0 and sin(alpha1) != 0 (Case 3 in Craig)
     */
//    std::vector<double> solveTheta3Case3(double r, double z) const;


//    double solveTheta2(double r, double z, double theta3) const;

    std::vector<double> solveTheta2Case1(double z, double theta3) const;
    std::vector<double> solveTheta2Case2(double r, double theta3) const;

    double solveTheta1(double x, double y, double theta2, double theta3) const;

    /*
     * Variables used for calculating
     * Defined here to avoid allocating memory for them all the time
     */
//    mutable double a,b,c,d,e;

    mutable double alpha0, a0, calpha0, salpha0, d1;
    mutable double alpha1, a1, calpha1, salpha1, d2;
    mutable double alpha2, a2, calpha2, salpha2, d3;
    mutable double alpha3, a3, calpha3, salpha3, d4;
    mutable double alpha4, a4, calpha4, salpha4, d5;
    mutable double alpha5, a5, calpha5, salpha5, d6;

    /**
     * @brief 判断可否通过该逆解器求解
     *
     * 判断条件为: <br>
     * @f$ \begin{array}{ccc}
     *  a_4=0 & d_4=0\\
     *  a_5=0 & \left (a_1=0\ or\ sin(\alpha_1) = 0\right )
     *  \end{array}
     *  @f$
     * @retval true 成功
     * @retval flase 失败
     */
    bool isValid() const;
	virtual ~PieperSolver();
private:
	/**
	 * @brief Modified DH 参数表
	 */
	robot::model::DHTable _dHTable;
//    robot::math::HTransform3D<> _baseTdhRef;

	/**
	 * @brief 从基座到0关节的变换矩阵
	 */
    robot::math::HTransform3D<> _0Tbase;

    /**
     * @brief 从末端执行器到6关节的变换矩阵
     */
    robot::math::HTransform3D<> _endTjoint6;
};

/** @} */
} /* namespace ik */
} /* namespace robot */

#endif /* PIEPERSOLVER_H_ */
