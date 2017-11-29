/**
 * @brief LeastSquare.h
 * @date Nov 15, 2017
 * @author a1994846931931
 */

#ifndef LEASTSQUARE_H_
#define LEASTSQUARE_H_

# include <vector>

using std::vector;

namespace robot {
namespace math {

/**
 * @addtogroup math
 * @{
 */

/**
 * @brief 最小二乘法计算类
 */
class LeastSquare {
public:
	LeastSquare();
	virtual ~LeastSquare();
public:
	/**
	 * @brief 矩阵最小二乘法求解
	 * @param A [in] @$ A @$
	 * @param b [in] @$ b @$
	 * @param X [out] @$ X @$
	 * @retval true 正确求解
	 * @retval false 无法求解
	 */
	static bool fromMatrix(const vector<vector<double> > &A, const vector<double> &b, vector<double> &X);
};

/** @} */

} /* namespace model */
} /* namespace robot */

#endif /* LEASTSQUARE_H_ */
