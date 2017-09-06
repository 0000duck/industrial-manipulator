/**
 * @brief 单位四元数类Quaternion
 * @date Aug 24, 2017
 * @author a1994846931931
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

//# include "Vector3D.h"
# include "Rotation3D.h"
# include "HTransform3D.h"

namespace robot {
namespace math {

/** @addtogroup math
 * @brief 四元数运算
 * @{
 */

/**
 * @brief 四元数类
 * 用于计算旋转矩阵变化，包括四则运算、矩阵转化
 */
class Quaternion {
/**
* @brief 四元数三角函数表示
* theta为旋转角度、n为单位方向向量
*
*/
	typedef struct{
		double theta;
		Vector3D<double> n;
	} rotVar;
public:
	/**
	 * @brief 默认构造函数
	 * 默认r=1；i=0；j=0；k=0
	 *
	 */
	Quaternion();
	/**
	 * @brief 四元数赋值构造函数
	 * @param r[in]
	 * @param i[in]
	 * @param j[in]
	 * @param k[in]
	 *
	 */
	Quaternion(double, double, double, double);
	/**
	 * @brief 将四个数值赋值给一个四元数
	 * @param QuatA[in]
	 */
	Quaternion(const Quaternion&);
	/**
	* @brief 从旋转矩阵变换到四元数
	* @param Rotation3D<double>& rot [in]
	*/
	Quaternion(const Rotation3D<double>&);
	/**
	 * @brief 从变换矩阵变换到四元数
	 * @param HTransform3D<double>& tran [in]
	 */
	Quaternion(const HTransform3D<double>&);
	/**
	 * @brief 从三角函数四元数转化到复数形式
	 * @param theta [in]
	 * @param n [in]
	 */
	Quaternion(double theta, const Vector3D<double>& n);
	/**
	 * @brief 四元数加法，对应项相加
	 * @param QuatA [in]
	 */
	Quaternion operator+(const Quaternion&) const;
	/**
	 * @brief 四元数减法，对应项相减
	 * @return 返回一个四元数
	 * @param QuatA [in]
	 */
	Quaternion operator-(const Quaternion&) const;
	/**
	* @brief 四元数乘法，对应项相乘
	* @return 返回一个四元数
	* @param QuatA [in]
	*/
	Quaternion operator*(const Quaternion&) const;
	/**
	* @brief 四元数赋值加法，对应项相加并赋值
	* @param QuatA [in]
	*/
	void operator+=(const Quaternion&);
	/**
		* @brief 四元数赋值减法，对应项相减并赋值
		* @param QuatA [in]
		*/
	void operator-=(const Quaternion&);
	/**
		* @brief 四元数赋值乘法，对应项相乘并赋值
		* @param QuatA [in]
		*/
	void operator*=(const Quaternion&);
	/**
		* @brief 赋值运算，将一个四元数赋值给另一个四元数
		* @param QuatA [in]
		*/
	void operator=(const Quaternion&);
	/**
		* @brief 判断是否相等
		* @return 1相等，0不等
		* @param QuatA [in]
		*/
	bool operator==(const Quaternion&) const;
	/**
		* @brief 判断是否相等
		* @return 1不等，0相等
		* @param QuatA [in]
		*/
	bool operator!=(const Quaternion&) const;
	/**
	* @brief 获取参数r
	* @return 返回参数r
	*/
	double r() const;
	/**
		* @brief 获取参数i
		* @return 返回参数i
		*/
	double i() const;
	/**
		* @brief 获取参数j
		* @return 返回参数j
		*/
	double j() const;
	/**
		* @brief 获取参数k
		* @return 返回参数k
		*/
	double k() const;
	/**
		* @brief 共轭四元数
		* @return 返回四元数
		*/
	Quaternion conjugate() const;
	/**
		* @brief 四元数的范数，开平方形式
		* @return double类型的数值
		*/
	double norm() const;
	/**
		* @brief 四元素的规范化
		*/
	void normalize();
	/**
		* @brief 从复数形式的四元数转化到旋转矩阵
		* @return 返回旋转矩阵
		*/
	robot::math::Rotation3D<double> toRotation3D() const;
	/**
		* @brief 从复数形式的四元数转化到变换矩阵
		* @return 返回变换矩阵
		*/
	robot::math::HTransform3D<double> toHTransform3D() const;
	/**
		* @brief 将复数形式的四元数转化为三角函数形式
		* @return 返回结构体rotVar
		*/
	rotVar getRotationVariables() const;
	/**
		* @brief 输出四元数
		*/
	void print() const;
public:
	/**
		* @brief 由DH参数得到四元数值
		* @return 返回一组四元数
		* @param alpha [in]
		* @param theta [in]
		*/
	static Quaternion DH(double alpha, double theta);
	virtual ~Quaternion();
private:
	double _r, _i, _j, _k;
};

/** @} */
} /* namespace math */
} /* namespace robot */

#endif /* QUATERNION_H_ */
