/**
 * @brief 变换矩阵模板类HTransform3D<>
 * @date Aug 10, 2017
 * @author a1994846931931
 */

#ifndef HTRANSFORM3D_H_
#define HTRANSFORM3D_H_

# include "Rotation3D.h"
# include "Vector3D.h"
# include <iostream>
# include <assert.h>
# include <math.h>
# include "../common/printAdvance.h"

namespace robot {
namespace math {

/** @addtogroup math
 * @brief 包括变换矩阵, 四元数, 数组等.
 *
 * 包括的类有:
 * 1. HTransform3D: 齐次变换矩阵
 * 2. Rotation3D: 旋转矩阵
 * 3. Vector3D: 三维向量
 * 4. Q: 一位数组
 * 5. Quaternion: 单位四元数
 * @{
 */

/**
 * @brief 4X4的齐次变换矩阵
 *
 * 用于描述位姿的矩阵. 可以进行+, -, *, +=, -=, *=, 求逆等一系列操作.
 */
template<typename T=double>
class HTransform3D {
public:
	/**
	 * @brief 默认构造函数.
	 *
	 * 默认位移为0, 旋转为单位矩阵. 即:
	 * @f$
	 * \left[
	 * \begin{array}{cccc}
	 * 1 & 0 & 0 & 0 \\
	 * 0 & 1 & 0 & 0 \\
	 * 0 & 0 & 1 & 0 \\
	 * 0 & 0 & 0 & 1
	 * \end{array}
	 * \right]
	 * @f$
	 */
	HTransform3D() :
    _vec(),
    _rot(Rotation3D<T>::identity())
	{}

	/**
	 * @brief 显式给出所有参数的构造函数
	 * @param r11 [in]
	 * @param r12 [in]
	 * @param r13 [in]
	 * @param r14 [in]
	 * @param r21 [in]
	 * @param r22 [in]
	 * @param r23 [in]
	 * @param r24 [in]
	 * @param r31 [in]
	 * @param r32 [in]
	 * @param r33 [in]
	 * @param r34 [in]
	 *
	 * 构造的矩阵如下:
	 * @f$
	 * \left[
	 * \begin{array}{cccc}
	 * r11 & r12 & r13 & r14 \\
	 * r21 & r22 & r23 & r24 \\
	 * r31 & r32 & r33 & r34 \\
	 * 0 & 0 & 0 & 1
	 * \end{array}
	 * \right]
	 * @f$
	 */
	HTransform3D(
			T r11, T r12, T r13, T r14,
			T r21, T r22, T r23, T r24,
			T r31, T r32, T r33, T r34) :
	_vec(r14, r24, r34),
	_rot(r11, r12, r13, r21, r22, r23, r31, r32, r33)
	{}

	/**
	 * @brief 根据位移和旋转矩阵构造
	 * @param d [in] 位移
	 * @param R [in] 旋转矩阵
	 *
	 * 构造的矩阵如下:
	 * @f$
	 * \left[
	 * \begin{array}{cc}
	 * \mathbf{R} & \mathbf{d} \\
	 * 0 & 1
	 * \end{array}
	 * \right]
	 * @f$
	 */
	HTransform3D(const Vector3D<T>& d, const Rotation3D<T>& R) :
	_vec(d),
	_rot(R)
	{}

	/**
	 * @brief 根据旋转矩阵构造
	 * @param d [in] 位移
	 * @param R [in] 旋转矩阵
	 *
	 * 构造的矩阵如下:
	 * @f$
	 * \left[
	 * \begin{array}{cc}
	 * \mathbf{R} & 0 \\
	 * 0 & 1
	 * \end{array}
	 * \right]
	 * @f$
	 */
	explicit HTransform3D(const Rotation3D<T>& R) :
	_vec(0, 0, 0),
	_rot(R)
	{}

	/**
	 * @brief 根据位移和旋转矩阵构造
	 * @param d [in] 位移
	 * @param R [in] 旋转矩阵
	 *
	 * 构造的矩阵如下:
	 * @f$
	 * \left[
	 * \begin{array}{cc}
	 * \mathbf{I^3} & \mathbf{d} \\
	 * 0 & 1
	 * \end{array}
	 * \right]
	 * @f$
	 */
	explicit HTransform3D(const Vector3D<T>& d) :
	_vec(d),
	_rot(Rotation3D<T>::identity())
	{}

	/**
	 * @brief 变换矩阵相乘
	 * @param tran [in] 右手边的被乘数
	 * @return 表达如下:
	 * @f$
	 * \mathbf{T_1}\cdot\mathbf{T_2} =
	 * \left[ \begin{array}{cc}
	 * \mathbf{R_1} & \mathbf{d_1} \\
	 * 0 & 1
	 * \end{array}
	 * \right] \cdot \left[
	 * \begin{array}{cc}
	 * \mathbf{R_2} & \mathbf{d_2} \\
	 * 0 & 1
	 * \end{array}
	 * \right] = \left[
	 * \begin{array}{cc}
	 * \mathbf{R_1}\mathbf{R_2} & \mathbf{R_1}\mathbf{d_2} + \mathbf{d_1} \\
	 * 0 & 1
	 * \end{array} \right]
	 * @f$
	 */
	const HTransform3D<T> operator* (const HTransform3D<T>& tran) const
	{
		return HTransform3D<T>(_rot*tran._vec + _vec, _rot*tran._rot);
	}

	/**
	 * @brief 变换矩阵相乘
	 * @param tran [in] 右手边的被乘矩阵
	 * @return 表达如下:
	 * @f$
	 * \left[ \begin{array}{cc}
	 * \mathbf{R_1} & \mathbf{d_1} \\
	 * 0 & 1
	 * \end{array}
	 * \right] \cdot \left[
	 * \begin{array}{cc}
	 * \mathbf{d_2} \\
	 * 1
	 * \end{array}
	 * \right]  = \left[
	 * \begin{array}{cc}
	 * \mathbf{R_1}\mathbf{d_2} + \mathbf{d_1} \\
	 * 1
	 * \end{array} \right]
	 * @f$
	 */
	const Vector3D<T> operator* (const Vector3D<T>& vec) const
	{
		return _rot*vec + _vec;
	}

	/**
	 * @brief *=操作
	 * @param tran  [in] 右手边的被乘矩阵
	 * @see robot::math::HTransform3D<T>::operator*
	 */
	void operator*= (const HTransform3D<T>& tran)
	{
		_vec += _rot*tran.getPosition();
		_rot *= tran.getRotation();
	}

	/**
	 * @brief =赋值操作
	 * @param tran [in] 赋值矩阵
	 */
	void operator=(const HTransform3D<T>& tran)
	{
		_rot = tran.getRotation();
		_vec = tran.getPosition();
	}

	/**
	 * @brief 判断是否相等
	 * @param tran [in] 比较矩阵
	 *
	 * 分别判断旋转矩阵与位移向量是否相等, 判断的实施方法参见各自的定义.
	 */
	bool operator==(const HTransform3D<T>& tran) const
	{
		if (_rot != tran.getRotation() || _vec != tran.getPosition())
			return false;
		return true;
	}

	/**
	 * @brief 变换矩阵求逆
	 * @return
	 * 变换矩阵的逆就是它的倒置. <br>
	 * 即:
	 * @f$
	 * \mathbf{T}^{-1} = \mathbf{T}^*
	 * @f$
	 */
	const HTransform3D<T> inverse() const
	{
		return HTransform3D<T>(-(_rot.inverse()*_vec), _rot.inverse());
	}

	/**
	 * @brief 取值操作
	 * @param row [in] 获取值所在的行数(从0开始)
	 * @param col [in] 获取值所在的列数(从0开始)
	 */
	const T operator()(int row, int col) const
	{
		assert(row < 3);
		assert(col < 4);
		if (col < 3)
			return _rot(row, col);
		else
			return _vec(row);
	}

	/**
	 * @brief 根据显式给出的参数设置旋转矩阵的值
	 *
	 * 该操作的好处是改变矩阵的值而不改变它的地址
	 */
	inline void setRotation(
			const T& r11, const T& r12, const T& r13,
			const T& r21, const T& r22, const T& r23,
			const T& r31, const T& r32, const T& r33)
	{
		_rot.setRotation(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	}

	/**
	 * @brief 根据显式给出的参数设置位移向量的值
	 *
	 * 该操作的好处是改变矩阵的值而不改变它的地址
	 */
	inline void setVector(const T& v1, const T& v2, const T& v3)
	{
		_vec.setVector(v1, v2, v3);
	}

	/**
	 * @brief 获取旋转矩阵
	 * @return
	 */
	inline const Rotation3D<T>& getRotation() const
	{
		return _rot;
	}

	/**
	 * @brief 获取位移向量
	 * @return
	 */
	inline const Vector3D<T>& getPosition() const
	{
		return _vec;
	}

	/**
	 * @brief显式地更新变换矩阵的值
	 *
	 * 该操作的好处是改变矩阵的值而不改变它的地址
	 */
	inline void update(
			const T& r11, const T& r12, const T& r13, const T& r14,
			const T& r21, const T& r22, const T& r23, const T& r24,
			const T& r31, const T& r32, const T& r33, const T& r34)
	{
		_rot.setRotation(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		_vec.setVector(r14, r24, r34);
	}

	/**
	 * @brief 生成默认变换矩阵
	 *
	 * 生成一个单位旋转举证和0位移向量构成的变换矩阵
	 */
	static const HTransform3D<T>& identity(){
		static const HTransform3D<T> id(
			Vector3D<T>(0, 0, 0),
			Rotation3D<T>::identity());
		return id;
	}

	/**
	 * @brief 根据DH参数构造变换矩阵
	 *
	 * 依据DH参数(Modified DH)构造b变换矩阵
	 * @param alpha [in] @f$ \alpha @f$
	 * @param a [in] @f$ a @f$
	 * @param d [in] @f$ d @f$
	 * @param theta [in] @f$ \theta @f$
	 * @return 构造的变换矩阵为:
	 * @f$ \left[ \begin{array}{cccc}
     * cos(\theta) & -sin(\theta) & 0 & a \\
     * sin(\theta)*cos(\alpha) & cos(\theta)*cos(\alpha) & -sin(\alpha) & -d*sin(\alpha) \\
     * sin(\theta)*sin(\alpha) & cos(\theta)*sin(\alpha) & cos(\alpha) & d*cos(\alpha)) \\
	 * 0 & 0 & 0 & 1
	 * \end{array} \right]
	 * @f$
	 */
	static const HTransform3D<T> DH(const T alpha, const T a, const T d, const T theta)
	{
		double st=sin(theta); double ct=cos(theta);double sa=sin(alpha);double ca=cos(alpha);
		return robot::math::HTransform3D<double>(
				ct, -st, 0, a,
				st*ca, ct*ca, -sa, -d*sa,
				st*sa, ct*sa, ca, d*ca);
	}

	/**
	 * @brief 构造由DH参数制定的变换矩阵关于theta的求导形式；
	 * 依据DH参数(Modified DH)构造b变换矩阵
	 * @param alpha [in] @f$ \alpha @f$
	 * @param a [in] @f$ a @f$
	 * @param d [in] @f$ d @f$
	 * @param theta [in] @f$ \theta @f$
	 * @return 构造的变换矩阵为:
	 * @f$ \left[ \begin{array}{cccc}
	 * -sin(\theta) & -cos(\theta) & 0 & 0 \\
	 * cos(\theta)*cos(\alpha) & -sin(\theta)*cos(\alpha) & 0 & 0 \\
	 * cos(\theta)*sin(\alpha) & -sin(\theta)*sin(\alpha) & 0 & 0 \\
	 * 0 & 0 & 0 & 1
	 * \end{array} \right]
	 * @f$
	 */
	static const HTransform3D<T> dDH(const T alpha, const T a, const T d, const T theta)
	{
		double st=sin(theta); double ct=cos(theta);double sa=sin(alpha);double ca=cos(alpha);
		return HTransform3D<double>(
				-st, -ct, 0, 0,
				ct*ca, -st*ca, 0, 0,
				ct*sa, -st*sa, 0, 0);
	}

	/**
	 * @brief 格式化打印输出
	 */
	void print() const{
		cout.precision(4);
		cout << setfill('_') << setw(56) <<  "_" << endl;
		for (int i = 0; i<3; i++)
			cout << setfill(' ') << setw(12) << _rot(i, 0) << " |"
			<< setfill(' ') << setw(12) << _rot(i, 1) << " |"
			<< setfill(' ') << setw(12) << _rot(i, 2) << " |"
			<< setfill(' ') << setw(12) << _vec(i) << '\n';
		cout << setfill(' ') << setw(12) << "0" << " |"
				<< setfill(' ') << setw(12) << "0" << " |"
				<< setfill(' ') << setw(12) << "0" << " |"
				<< setfill(' ') << setw(12) << "1" << '\n';
	}
	virtual ~HTransform3D(){}
private:
	/**
	 * @brief 位移向量
	 */
	Vector3D<T> _vec;

	/**
	 * @brief 旋转矩阵
	 */
	Rotation3D<T> _rot;
};

/** @} */
} /* namespace math */
} /* namespace robot */

#endif /* HTRANSFORM3D_H_ */
