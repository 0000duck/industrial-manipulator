/**
 * @brief DH参数类
 * @date Aug 17, 2017
 * @author a1994846931931
 */

#ifndef DHPARAMETERS_H_
#define DHPARAMETERS_H_

namespace robot {
namespace model {

/** @addtogroup model
 * @{
 */

/**
 * @brief DH参数类
 *
 * 表示DH表中的一行, @f$ [\alpha, a, d, \theta] @f$
 */
class DHParameters {
public:
	/**
	 * @brief 构造函数
	 * @param alpha @f$ \alpha @f$
	 * @param a @f$ a @f$
	 * @param d @f$ d @f$
	 * @param theta @f$ \theta @f$
	 *
	 * @f$ [\alpha, a, d, \theta] @f$
	 */
	DHParameters(double alpha, double a, double d, double theta);

	/** @brief 获取 @f$ \alpha @f$ */
	double alpha() const;

	/** @brief 获取 @f$ a @f$ */
	double a() const;

	/** @brief 获取 @f$ d @f$ */
	double d() const;

	/** @brief 获取 @f$ \theta @f$ */
	double theta() const;

	/** @brief 格式化打印 */
	void print() const;
	virtual ~DHParameters();
private:
	/** @brief @f$ \alpha @f$ */
	double _alpha;

	/** @brief @f$ a @f$ */
	double _a;

	/** @brief @f$ d @f$ */
	double _d;

	/** @brief @f$ \theta @f$ */
	double _theta;
};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* DHPARAMETERS_H_ */
