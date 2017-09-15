/**
 * @brief 齐次三维向量类Vector3D
 * @date Aug 10, 2017
 * @author a1994846931931
 */

#ifndef VECTOR3D_H_
#define VECTOR3D_H_

# include "Rotation3D.h"
# include <iostream>
# include "../common/printAdvance.h"
# include "math.h"

namespace robot {
namespace math {

/** @addtogroup math
 * @{
 */

/**
 * @brief 3X1向量模板类
 *
 * 用以描述坐标系的位移, 可以进行+, -, dot, cross等操作
 */
template<typename T=double>
class Vector3D {
public:
	/**
	 * @brief 默认构造函数
	 *
	 * 构造为3X1的0向量:
	 * @f$
	 * \left[ \begin{array}{ccc}
	 * 0 \\ 0 \\ 0
	 * \end{array} \right]
	 * @f$
	 */
	Vector3D()
	{
		_v[0] = _v[1] = _v[2] = 0;
	}

	/**
	 * @brief 显式给出所有数值的构造函数
	 * @param v1 [in]
	 * @param v2 [in]
	 * @param v3 [in]
	 * 构造的向量如下:
	 * @f$
	 * \left[
	 * \begin{array}{cccc}
	 * v1 \\
	 * v2 \\
	 * v3
	 * \end{array}
	 * \right]
	 * @f$
	 */
	Vector3D(T v1, T v2, T v3)
	{
		_v[0] = v1;
		_v[1] = v2;
		_v[2] = v3;
	}

	/**
	 * @brief 通过从另一个向量复制进行构造
	 * @param vSource [in] 复制源
	 */
	Vector3D(const Vector3D<T>& vSource)
	{
		_v[0] = vSource(0);
		_v[1] = vSource(1);
		_v[2] = vSource(2);
	}

	/**
	 * @brief 获取向量的长度
	 * @return
	 * @f$
	 * Length = \sqrt{v_1^2 + v_2^2 + v_3^2}
	 * @f$
	 */
	double getLengh() const
	{
		return sqrt(_v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]);
	}

	/**
	 * @brief 取值操作
	 * @param i [in] 索引位置
	 * @return 返回
	 * @f$
	 * \mathbf{f}(i)
	 * @f$
	 */
	const T operator()(int i) const
	{
		return _v[i];
	}

	/**
	 * @brief 取值操作
	 * @param i [in] 索引位置
	 * @return 返回
	 * @f$
	 * \mathbf{f}(i)
	 * @f$
	 * 处的引用
	 */
	T& operator[](int i)
	{
		return _v[i];
	}

	/**
	 * @brief 复制操作
	 * @param vSource [in] 复制源
	 */
	void operator=(const Vector3D<T>& vSource)
	{
		_v[0] = vSource(0);
		_v[1] = vSource(1);
		_v[2] = vSource(2);
	}

	/**
	 * @brief 向量+=操作
	 * @param vSource [in] 加数
	 */
	void operator+=(const Vector3D<T>& vSource)
	{
		_v[0] += vSource(0);
		_v[1] += vSource(1);
		_v[2] += vSource(2);
	}

	/**
	 * @brief 判断相等
	 * @param vSource [in] 比较的向量
	 *
	 * 考虑到浮点计算的误差, 两个向量不可能完全相等, 因此判断条件修改为, 所有对应位置上的数相差不超过1e-12
	 * @return 符合判断条件时为true.
	 */
	bool operator==(const Vector3D<T>& vSource) const
	{
		if (fabs(_v[0] - vSource(0)) > 1e-12 or fabs(_v[1] - vSource(1)) > 1e-12 or fabs(_v[2] - vSource(2)) > 1e-12)
			return false;
		return true;
	}

	/**
	 * @brief 判断不相等
	 * @param vSource [in] 比较的向量
	 *
	 * 考虑到浮点计算的误差, 两个向量不可能完全相等, 因此判断条件修改为, 存在对应位置上的数相差超过1e-12
	 * @return 符合判断条件时为true.
	 */
	bool operator!=(const Vector3D<T>& vSource) const
	{
		return (!this->operator ==(vSource));
	}

	/**
	 * @brief 负操作
	 * @return
	 * @f$
	 * - \left[ \begin{array}{ccc}
	 * v_1 \\ v_2 \\ v_3
	 * \end{array} \right]
	 * =
	 * \left[ \begin{array}{ccc}
	 * -v_1 \\ -v_2 \\ -v_3
	 * \end{array} \right]
	 * @f$
	 */
	Vector3D<T> operator-() const
	{
		return Vector3D<T>(
				-_v[0], -_v[1], -_v[2]);
	}

	/**
	 * @brief 向量相加操作
	 * @param vec [in] 加数
	 * @return 两个向量对应位置元素相减得到的新向量
	 */
	Vector3D<T> operator+(Vector3D<T> vec) const
	{
		return Vector3D<T>(
				vec(0) + _v[0], vec(1) + _v[1], vec(2) + _v[2]);
	}

	/**
	 * @brief 向量相减操作
	 * @param vec [in] 减数
	 * @return 两个向量对应位置元素相减得到的新向量
	 */
	Vector3D<T> operator-(const Vector3D<T>& vec) const
	{
		return Vector3D<T>(
				_v[0] - vec(0), _v[1] - vec(1), _v[2] - vec(2));
	}

	/**
	 * @brief 向量与常亮相乘操作
	 * @param factor [in] 相乘的常量
	 * @return 向量每个元素与factor相乘得到的新向量
	 */
	Vector3D<T> operator*(T factor) const
	{
		return Vector3D<T>(
				_v[0]*factor, _v[1]*factor, _v[2]*factor);
	}

	/**
	 * @brief 向量与常亮相除操作
	 * @param factor [in] 相除的常量
	 * @return 向量每个元素与factor相除得到的新向量
	 */
	Vector3D<T> operator/(T factor) const
	{
		return Vector3D<T>(
				_v[0]/factor, _v[1]/factor, _v[2]/factor);
	}

	/**
	 * @brief 向量点乘
	 * @param a [in]
	 * @param b [in]
	 * @return
	 * @f$
	 * \left[ \begin{array}{ccc}
	 * a_1 \\ a_2 \\ a_3
	 * \end{array} \right]
	 * \cdot
	 * \left[ \begin{array}{ccc}
	 * b_1 \\ b_2 \\ b_3
	 * \end{array} \right]
	 * =
	 * \left[ \begin{array}{ccc}
	 * a_1b_1 \\ a_2b_2 \\ a_3b_3
	 * \end{array} \right]
	 * @f$
	 */
	static T dot(const Vector3D<T>& a, const Vector3D<T>& b)
	{
		return (a(0)*b(0) + a(1)*b(1) + a(2)*b(2));
	}

	/**
	 * @brief 向量复制操作
	 * @param v1 [in]
	 * @param v2 [in]
	 * @param v3 [in]
	 */
	void setVector(const T& v1, const T& v2, const T& v3)
	{
		_v[0] = v1;
		_v[1] = v2;
		_v[2] = v3;
	}

	/**
	 * @brief 叉乘操作
	 * @param a [in] 左乘数
	 * @param b [in] 右乘数
	 * @return
	 * @f$
	 * \left[ \begin{array}{ccc}
	 * a_1 \\ a_2 \\ a_3
	 * \end{array} \right]
	 * \times
	 * \left[ \begin{array}{ccc}
	 * b_1 \\ b_2 \\ b_3
	 * \end{array} \right]
	 * =
	 * \left| \begin{array}{ccc}
	 * \mathbf{i} & \mathbf{j} & \mathbf{k} \\
	 * a_1 & a_2 & a_3 \\
	 * b_1 & b_2 & b_3
	 * \end{array} \right|
	 * =
	 * \left[ \begin{array}{ccc}
	 * a_2b_3 - a_3b_2 \\ a_3b_1 - a_1b_3 \\ a_1b_2 - a_2b_1
	 * \end{array} \right]
	 * @f$
	 */
	static Vector3D<T> cross(const Vector3D<T>& a, const Vector3D<T>& b)
	{
		return Vector3D<T>(a(1)*b(2) - a(2)*b(1),
				a(2)*b(0) - a(0)*b(2),
				a(0)*b(1) - a(1)*b(0));
	}

	/**
	 * @brief 向量规范化
	 * @param a [in] 源向量
	 * @return
	 * @f$
	 * \frac{\mathbf{V}}{|\mathbf{V}|}
	 * @f$
	 */
	static Vector3D<T> normalize(const Vector3D<T>& a)
	{
		T len = a.getLengh();
		return Vector3D<T>(a(0)/len, a(1)/len, a(2)/len);
	}

	/**
	 * @brief 格式化打印
	 */
	void print() const
	{
		cout.precision(4);
		cout << setfill('_') <<setw(42)<< "_" << endl;
		std::cout << setfill(' ') << setw(12) << _v[0] << " |" << setfill(' ') << setw(12) << _v[1] << " |" << setfill(' ') << setw(12) << _v[2] << std::endl;
	}
	virtual ~Vector3D(){}
private:
	/**
	 * @brief 保存的3X1数组
	 */
	T _v[3];
};

/** @} */
} /* namespace math */
} /* namespace robot */

#endif /* VECTOR3D_H_ */
