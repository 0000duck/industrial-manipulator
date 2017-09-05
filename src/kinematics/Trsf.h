/**
 * @brief Trsf类
 * @date Aug 11, 2017
 * @author a1994846931931
 */

#ifndef TRSF_H_
#define TRSF_H_

# include "../math/HTransform3D.h"

using namespace robot::math;

namespace robot {
namespace kinematic {

/** @addtogroup kinematics
 * @{
 */

/**
 * @brief Trsf类, 参考VAL3中的Trsf变量.
 *
 * 用x, y, z, rx, ry, rz表示一个位姿.
 * (x, y, z)表示平移量, (rx, ry, rz)表示旋转量, 旋转的实现形式为, 先后绕固定坐标系的x轴, y轴和z轴旋转rx, ry, rz角度. 即:<br>
 *
 * @f$ Rotation(rx, ry, rz) = RotX(rx)*RotY(ry)*RotZ(rz) @f$
 */
class Trsf {
public:
	/**
	 * @brief 默认构造函数
	 *
	 * @f$ x=y=z=rx=ry=rz=0 @f$
	 */
	Trsf();

	/**
	 * @brief 完整构造函数
	 * @param x  [in] 位置x
	 * @param y  [in] 位置y
	 * @param z  [in] 位置z
	 * @param rx [in]
	 * @param ry [in]
	 * @param rz [in]
	 */
	Trsf(double x, double y, double z, double rx, double ry, double rz);

	/**
	 * @brief 获取变换矩阵
	 * @return 如果变换矩阵已近生成, 则直接返回, 若无, 调用doGetTransform后再返回
	 * @see doGetTransform
	 */
	robot::math::HTransform3D<double>& getTransform();
	virtual ~Trsf();
private:
	/**
	 * @brief 位移量x
	 */
	double _x;

	/**
	 * @brief 位移量y
	 */
	double _y;

	/**
	 * @brief 位移量z
	 */
	double _z;

	/**
	 * @brief 旋转量rx
	 */
	double _rx;

	/**
	 * @brief 旋转量ry
	 */
	double _ry;

	/**
	 * @brief 旋转量rz
	 */
	double _rz;

	/**
	 * @brief 变换矩阵
	 */
	robot::math::HTransform3D<double> _tran;

	/**
	 * @brief 标识是否需要重新生成变换矩阵
	 */
	bool _isTranformChanged;
private:
	/**
	 * 变换矩阵由以下公式给出:
	 * @f$
	 * \left[
	 * \begin{array}{cccc}
	 * cos(ry)*cos(rz) & -cos(ry)*sin(rz) & sin(ry) & x\\
	 * sin(rx)*sin(ry)*cos(rz) + cos(rx)*sin(rz) & -sin(rx)*sin(ry)*sin(rz) + cos(rx)*cos(rz) & -sin(rx)*cos(ry) & y\\
	 * -cos(rx)*sin(ry)*cos(rz) + sin(rx)*sin(rz) & cos(rx)*sin(ry)*sin(rz) + sin(rx)*cos(rz) & cos(rx)*cos(ry) & z\\
	 * 0 & 0 & 0 & 1
	 * \end{array}
	 * \right]
	 * @f$
	 */
	void doGetTransform();
};

/** @} */
} /* namespace kinematic */
} /* namespace robot */

#endif /* TRSF_H_ */
