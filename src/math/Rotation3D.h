/**
 * @brief 旋转矩阵模板类Rotation3D<>
 * @date Aug 10, 2017
 * @author a1994846931931
 */

#ifndef ROTATION3D_H_
#define ROTATION3D_H_

# include <iostream>
# include <limits>
# include <math.h>
# include "Vector3D.h"

using std::cout;

namespace robot {
namespace math {

/** @addtogroup math
 * @{
 */

/**
 * @brief 3X3旋转矩阵模板类
 *
 * 用以描述坐标系的旋转变换, 可以进行+, -, *等操作
 */
template<class T=double>
class Rotation3D {
public:
	/**
	 * @brief 默认构造函数
	 *
	 * 构造为3X3的单位矩阵:
	 * @f$
	 * \left[ \begin{array}{ccc}
	 * 1 & 0 & 0 \\
	 * 0 & 1 & 0 \\
	 * 0 & 0 & 1
	 * \end{array} \right]
	 * @f$
	 */
	Rotation3D(){
		_m[0][0] = 1;
		_m[0][1] = 0;
		_m[0][2] = 0;
		_m[1][0] = 0;
		_m[1][1] = 1;
		_m[1][2] = 0;
		_m[2][0] = 0;
		_m[2][1] = 0;
		_m[2][2] = 1;
	}

	/**
	 * @brief 完整地显示构造旋转矩阵
	 * @param r11 [in]
	 * @param r12 [in]
	 * @param r13 [in]
	 * @param r21 [in]
	 * @param r22 [in]
	 * @param r23 [in]
	 * @param r31 [in]
	 * @param r32 [in]
	 * @param r33 [in]
	 *
	 * 构造的矩阵如下:
	 * @f$
	 * \left[ \begin{array}{ccc}
	 * r11 & r12 & r13 \\
	 * r21 & r22 & r23 \\
	 * r31 & r32 & r33
	 * \end{array} \right]
	 * @f$
	 */
	Rotation3D(
		const T r11, const T r12, const T r13,
		const T r21, const T r22, const T r23,
		const T r31, const T r32, const T r33){
		_m[0][0] = r11;
		_m[0][1] = r12;
		_m[0][2] = r13;
		_m[1][0] = r21;
		_m[1][1] = r22;
		_m[1][2] = r23;
		_m[2][0] = r31;
		_m[2][1] = r32;
		_m[2][2] = r33;
	}

	/**
	 * @brief 用三个3X1的向量构造旋转矩阵
	 * @param i [in] 第1列向量
	 * @param j [in] 第2列向量
	 * @param k [in] 第3列向量
	 *
	 * 构造的矩阵如下:
	 * @f$
	 * \left[ \begin{array}{ccc}
	 * \\
	 * \mathbf{i} & \mathbf{j} & \mathbf{k}\\
	 *  & & \\
	 * \end{array} \right]
	 * @f$
	 */
	Rotation3D(
			const Vector3D<T>& i,
			const Vector3D<T>& j,
			const Vector3D<T>& k){
		_m[0][0] = i(0);
		_m[0][1] = j(0);
		_m[0][2] = k(0);
		_m[1][0] = i(1);
		_m[1][1] = j(1);
		_m[1][2] = k(1);
		_m[2][0] = i(2);
		_m[2][1] = j(2);
		_m[2][2] = k(2);
	}

	/**
	 * @brief 设置旋转矩阵数值
	 * @param r11 [in]
	 * @param r12 [in]
	 * @param r13 [in]
	 * @param r21 [in]
	 * @param r22 [in]
	 * @param r23 [in]
	 * @param r31 [in]
	 * @param r32 [in]
	 * @param r33 [in]
	 *
	 * 设置的矩阵如下:
	 * @f$
	 * \left[ \begin{array}{ccc}
	 * r11 & r12 & r13 \\
	 * r21 & r22 & r23 \\
	 * r31 & r32 & r33
	 * \end{array} \right]
	 * @f$
	 */
	void setRotation(
			T r11, T r12, T r13,
			T r21, T r22, T r23,
			T r31, T r32, T r33)
	{
		_m[0][0] = r11;
		_m[0][1] = r12;
		_m[0][2] = r13;
		_m[1][0] = r21;
		_m[1][1] = r22;
		_m[1][2] = r23;
		_m[2][0] = r31;
		_m[2][1] = r32;
		_m[2][2] = r33;
	}

	/**
	 * @brief 生成3X3的单位矩阵
	 * @return
	 * 构造的单位矩阵如下:
	 * @f$
	 * \left[ \begin{array}{ccc}
	 * 1 & 0 & 0 \\
	 * 0 & 1 & 0 \\
	 * 0 & 0 & 1
	 * \end{array} \right]
	 * @f$
	 */
	static const Rotation3D& identity()
	{
		static Rotation3D id(1,0,0,0,1,0,0,0,1);
		return id;
	}

	/**
	 * @brief 取值操作
	 * @param row [in] 取值的行数(从0开始)
	 * @param column [in] 取值的列数(从0开始)
	 * @return 返回row行colum列的值
	 */
	inline const T& operator()(int row, int column) const
	{
		return _m[row][column];
	}
//	inline T& operator[](int row, int column) const{
//			return _m[row][column];
//	}

	/**
	 * @brief 判断相等
	 * @param RotB [in] 判断是否相等的矩阵
	 *
	 * 考虑到浮点计算的误差, 两个矩阵不可能完全相等, 因此判断条件修改为, 所有对应位置上的数相差不超过1e-12
	 * @return 符合判断条件时为true.
	 */
	bool operator==(const Rotation3D<T>& RotB) const
	{
		for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
				if (fabs(_m[i][j] - RotB(i, j)) > 1e-12)
					return false;
		return true;
	}

	/**
	 * @brief 矩阵*=操作
	 * @param RotB [in] 乘数
	 */
	void operator*=(const Rotation3D<T> RotB)
	{
		double a[3];
		a[0] = _m[0][0]*RotB(0, 0) + _m[0][1]*RotB(1, 0) + _m[0][2]*RotB(2, 0);
		a[1] = _m[0][0]*RotB(0, 1) + _m[0][1]*RotB(1, 1) + _m[0][2]*RotB(2, 1);
		a[2] = _m[0][0]*RotB(0, 2) + _m[0][1]*RotB(1, 2) + _m[0][2]*RotB(2, 2);
		_m[0][0] = a[0]; _m[0][1] = a[1]; _m[0][2] = a[2];

		a[0] = _m[1][0]*RotB(0, 0) + _m[1][1]*RotB(1, 0) + _m[1][2]*RotB(2, 0);
		a[1] = _m[1][0]*RotB(0, 1) + _m[1][1]*RotB(1, 1) + _m[1][2]*RotB(2, 1);
		a[2] = _m[1][0]*RotB(0, 2) + _m[1][1]*RotB(1, 2) + _m[1][2]*RotB(2, 2);
		_m[1][0] = a[0]; _m[1][1] = a[1]; _m[1][2] = a[2];

		a[0] = _m[2][0]*RotB(0, 0) + _m[2][1]*RotB(1, 0) + _m[2][2]*RotB(2, 0);
		a[1] = _m[2][0]*RotB(0, 1) + _m[2][1]*RotB(1, 1) + _m[2][2]*RotB(2, 1);
		a[2] = _m[2][0]*RotB(0, 2) + _m[2][1]*RotB(1, 2) + _m[2][2]*RotB(2, 2);
		_m[2][0] = a[0]; _m[2][1] = a[1]; _m[2][2] = a[2];
	}

	/**
	 * @brief 与向量相乘并将结果赋值给向量
	 * @param vec [in] 相乘的向量
	 */
	void operator*=(Vector3D<T>& vec) const
	{
		double v0 = _m[0][0]*vec(0) + _m[0][1]*vec(1) + _m[0][2]*vec(2);
		double v1 = _m[1][0]*vec(0) + _m[1][1]*vec(1) + _m[1][2]*vec(2);
		vec[2] = _m[2][0]*vec(0) + _m[2][1]*vec(1) + _m[2][2]*vec(2);
		vec[0] = v0;
		vec[1] = v1;
	}

	/**
	 * @brief 矩阵赋值操作
	 * @param RotB [in] 赋值矩阵
	 */
	void operator=(const Rotation3D<T>& RotB)
	{
		_m[0][0] = RotB(0, 0);
		_m[0][1] = RotB(0, 1);
		_m[0][2] = RotB(0, 2);

		_m[1][0] = RotB(1, 0);
		_m[1][1] = RotB(1, 1);
		_m[1][2] = RotB(1, 2);

		_m[2][0] = RotB(2, 0);
		_m[2][1] = RotB(2, 1);
		_m[2][2] = RotB(2, 2);
	}

	/**
	 * @brief 判断不相等
	 * @param RotB [in] 判断是否相等的矩阵
	 *
	 * 考虑到浮点计算的误差, 两个矩阵不可能完全相等, 因此判断条件修改为, 存在对应位置上的数相差超过1e-12
	 * @return 符合判断条件时为true.
	 */
	bool operator!=(const Rotation3D<T>& RotB) const
	{
		return !(*this == RotB);
	}

	/**
	 * @brief 带精度参数的相等判断
	 * @param rot [in] 判断相等的数
	 * @param precision [in] 精度
	 *
	 * 判断条件为, 所有对应位置上的数相差不超过"precision"
	 * @return  符合判断条件时为true.
	 */
	bool equal(const Rotation3D<T>& rot, const T precision = std::numeric_limits<T>::epsilon()) const {
		for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
				if (fabs(_m[i][j] - rot(i,j)) > precision)
					return false;
		return true;
	}

	/**
	 * @brief 相乘操作
	 * @param a [in] 被乘数
	 * @param b [in] 乘数
	 * @return 返回两个矩阵相乘的结果
	 */
	static Rotation3D<T> multiply(const Rotation3D<T>& a, const Rotation3D<T>& b){
		return Rotation3D<T>(
				a(0, 0)*b(0, 0) + a(0, 1)*b(1, 0) + a(0, 2)*b(2, 0),
				a(0, 0)*b(0, 1) + a(0, 1)*b(1, 1) + a(0, 2)*b(2, 1),
				a(0, 0)*b(0, 2) + a(0, 1)*b(1, 2) + a(0, 2)*b(2, 2),

				a(1, 0)*b(0, 0) + a(1, 1)*b(1, 0) + a(1, 2)*b(2, 0),
				a(1, 0)*b(0, 1) + a(1, 1)*b(1, 1) + a(1, 2)*b(2, 1),
				a(1, 0)*b(0, 2) + a(1, 1)*b(1, 2) + a(1, 2)*b(2, 2),

				a(2, 0)*b(0, 0) + a(2, 1)*b(1, 0) + a(2, 2)*b(2, 0),
				a(2, 0)*b(0, 1) + a(2, 1)*b(1, 1) + a(2, 2)*b(2, 1),
				a(2, 0)*b(0, 2) + a(2, 1)*b(1, 2) + a(2, 2)*b(2, 2));
	}

	/**
	 * @brief 相减
	 * @param rot [in] 减数
	 * @return 返回矩阵相减的结果
	 */
	Rotation3D<T> operator-(const Rotation3D<T> rot) const
	{
		return Rotation3D<T>(
				_m[0][0] - rot(0, 0), _m[0][1] - rot(0, 1), _m[0][2] - rot(0, 2),
				_m[1][0] - rot(1, 0), _m[1][1] - rot(1, 1), _m[1][2] - rot(1, 2),
				_m[2][0] - rot(2, 0), _m[2][1] - rot(2, 1), _m[2][2] - rot(2, 2));
	}

	/**
	 * @brief 相加
	 * @param rot [in] 加数
	 * @return 返回矩阵相加的结果
	 */
	Rotation3D<T> operator+(const Rotation3D<T> rot) const
	{
		return Rotation3D<T>(
				_m[0][0] + rot(0, 0), _m[0][1] + rot(0, 1), _m[0][2] + rot(0, 2),
				_m[1][0] + rot(1, 0), _m[1][1] + rot(1, 1), _m[1][2] + rot(1, 2),
				_m[2][0] + rot(2, 0), _m[2][1] + rot(2, 1), _m[2][2] + rot(2, 2));
	}

	/**
	 * @brief 相乘
	 * @param rot [in] 乘数
	 * @return 返回矩阵相乘的结果
	 */
	Rotation3D<T> operator*(const Rotation3D<T> rot) const
	{
		return multiply(*this, rot);
	}

	/**
	 * @brief 矩阵与3数组相乘
	 * @param vec [in] 乘数
	 * @return 返回相乘得到的数组
	 */
	Vector3D<T> operator*(const Vector3D<T> vec) const
	{
		return Vector3D<T>(
				_m[0][0]*vec(0) + _m[0][1]*vec(1) + _m[0][2]*vec(2),
				_m[1][0]*vec(0) + _m[1][1]*vec(1) + _m[1][2]*vec(2),
				_m[2][0]*vec(0) + _m[2][1]*vec(1) + _m[2][2]*vec(2));
	}

	/**
	 * @brief 矩阵与常数相乘
	 * @param num [in] 乘数
	 * @return 返回相乘得到的旋转矩阵
	 */
	Rotation3D<T> operator*(const T& num) const
	{
		return Rotation3D<T>(
				_m[0][0]*num, _m[0][1]*num, _m[0][2]*num,
				_m[1][0]*num, _m[1][1]*num, _m[1][2]*num,
				_m[2][0]*num, _m[2][1]*num, _m[2][2]*num);
	}

	/**
	 * @brief 求逆操作
	 *
	 * 对于旋转矩阵, 矩阵的逆就是它的倒置.
	 * @return
	 * @f$
	 * \left[ \begin{array}{c}
	 * \mathbf{R^{-1}}
	 * \end{array} \right]
	 * =
	 * \left[ \begin{array}{c}
	 * \mathbf{R^*}
	 * \end{array} \right]
	 * @f$
	 */
	Rotation3D<T> inverse() const
	{
		return Rotation3D<T>(
				_m[0][0], _m[1][0], _m[2][0],
				_m[0][1], _m[1][1], _m[2][1],
				_m[0][2], _m[1][2], _m[2][2]);
	}

	/**
	 * @brief 构造由DH参数制定的旋转矩阵关于theta的求导形式；
	 * 依据DH参数(Modified DH)构造b旋转矩阵
	 * @param alpha [in] @f$ \alpha @f$
	 * @param a [in] @f$ a @f$
	 * @param d [in] @f$ d @f$
	 * @param theta [in] @f$ \theta @f$
	 * @return 构造的旋转矩阵为:
	 * @f$ \left[ \begin{array}{ccc}
	 * -sin(\theta) & -cos(\theta) & 0 \\
	 * cos(\theta)*cos(\alpha) & -sin(\theta)*cos(\alpha) & 0 \\
	 * cos(\theta)*sin(\alpha) & -sin(\theta)*sin(\alpha) & 0
	 * \end{array} \right]
	 * @f$
	 */
	static const Rotation3D<T> dDH(const T alpha, const T a, const T d, const T theta)
	{
		double st=sin(theta); double ct=cos(theta);double sa=sin(alpha);double ca=cos(alpha);
		return Rotation3D<double>(
				-st, -ct, 0,
				ct*ca, -st*ca, 0,
				ct*sa, -st*sa, 0);
	}

	/**
	 * @brief 格式化打印
	 */
	void print() const
	{
		cout.precision(4);
		cout << setfill('_') << setw(42) << "_" << endl;
		for (int i=0; i<3; i++)
			cout << setfill(' ') << setw(12) << _m[i][0] << " |"
			<< setfill(' ') << setw(12) << _m[i][1] << " |"
			<< setfill(' ') << setw(12) << _m[i][2] << '\n';
	}

	virtual ~Rotation3D(){}
private:
	/**
	 * @brief 保存的3X3数组
	 */
	T _m[3][3];
};

/** @} */
} /* namespace math */
} /* namespace robot */

#endif /* ROTATION3D_H_ */
