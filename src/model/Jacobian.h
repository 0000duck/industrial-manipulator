/**
 * @brief Jacobian类
 * @date Aug 24, 2017
 * @author a1994846931931
 */

#ifndef JACOBIAN_H_
#define JACOBIAN_H_

# include "../ext/Eigen/Dense"
# include "../math/Q.h"
# include <vector>

namespace robot {
namespace model {

/** @addtogroup model
 * @{
 */

/**
 * @brief 雅克比矩阵
 *
 * 雅克比矩阵为一个6Xn的矩阵, 雅克比矩阵与关节速度相乘可以得到一个6X1的速度向量: <br>
 * @f$ \mathbf{J}\cdot\dot\mathbf{q} = \mathbf{V} @f$ <br>
 * 其中: <br>
 * @f$ \mathbf{V} =
 * \left\{\begin{array}{cccccc}v_x \\ v_y \\ v_z \\ w_x \\ w_y \\ w_z
 * \end{array}\right\}@f$ <br>
 *
 * 一些具体实现使用了Eigen(版本3.3.4）
 * > http://eigen.tuxfamily.org/index.php?title=Main_Page
 */
class Jacobian {
public:
	/**
	 * @brief 默认构造函数
	 *
	 * 默认构造一个6X6, 元素为0的矩阵
	 */
	Jacobian();

	/**
	 * @brief 从二维数组生成的构造函数
	 *
	 * 每一个std::vector<double>元素代表雅克比矩阵中的一行. 构造时必须保证有6行.
	 */
	Jacobian(std::vector< std::vector<double> >);

	/**
	 * @brief 雅克比矩阵求逆
	 *
	 * 仅对6X6的雅克比矩阵有效. 具体实现参考Eigen.
	 */
	void doInverse();

	/**
	 * @brief 雅克比矩阵求逆
	 *
	 * 仅对6X6的雅克比矩阵有效. 具体实现参考Eigen.
	 * > https://eigen.tuxfamily.org/dox/classEigen_1_1Inverse.html
	 * @return 返回逆矩阵
	 */
	Jacobian inverse() const;

	/**
	 * @brief 求秩
	 *
	 * 使用采取列主元消去法(column pivoting)的QR分解(QR decomposition)秩显(rank revealing)算法.
	 * 它将一个矩阵 @f$ \mathbf{A} @f$ 分解为矩阵 @f$ \mathbf{P} @f$, @f$ \mathbf{Q} @f$ 和
	 * @f$ \mathbf{R} @f$ 使得 <br>
	 * @f$ \mathbf{A}\mathbf{P} =  \mathbf{Q}\mathbf{R}@f$ <br>
	 * @return 雅克比矩阵的秩
	 *
	 * 具体实现参考Eigen中的 Eigen::ColPivHouseholderQR<> 类.
	 * > https://eigen.tuxfamily.org/dox/classEigen_1_1ColPivHouseholderQR.html
	 */
	int rank() const;

	/**
	 * @brief 格式化打印
	 */
	void print() const;

	/**
	 * @brief 跟新数据
	 * @param j [in] 用于赋值的二维数组
	 */
	void update(std::vector< std::vector<double> > j);

	/**
	 * @brief 取值操作
	 * @param row [in] 取值的行数(从0开始)
	 * @param col [in] 取值的列数(从0开始)
	 * @return 返回row行col列的数
	 */
	double operator()(int row, int col) const;

	/**
	 * @brief 复制操作
	 * @param j [in] 复制的雅克比矩阵
	 */
	void operator=(const Jacobian& j);

	/**
	 * @brief 与robot::math::Q的乘法操作
	 *
	 * 用于实现雅克比矩阵或逆矩阵与关节速度或末端执行器速度的相乘操作
	 * @param Q [in] 关节速度或末端执行器速度向量
	 */
	robot::math::Q operator*(const robot::math::Q& Q) const;

	/** @brief 返回矩阵大小(列数) */
	int size() const;
	virtual ~Jacobian();
private:
	Eigen::MatrixXd getMatrix() const
	{
		return _j;
	}
private:
	/** @brief 矩阵大小(列数) */
	int _size;

	/** Eigen::MatrixXd 记录的矩阵 */
	Eigen::MatrixXd _j;
};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* JACOBIAN_H_ */
