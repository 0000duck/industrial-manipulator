/**
 * @brief Interpolator类
 * @date Sep 1, 2017
 * @author a1994846931931
 */

#ifndef INTERPOLATOR_H_
#define INTERPOLATOR_H_

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @brief 插补器.
 * @{
 */

/**
 * @brief 插补器接口
 *
 * 所有插补器一致的基类, 定义了公共的函数借口x(t), dx(t), ddx(t)和duration().
 * 参照具体类的实现
 */
template <class T>
class Interpolator {
public:
	Interpolator(){}
	virtual ~Interpolator(){}

	/**
	 * @brief t时刻的位置
	 * @param t [in] 时间t
	 * @return 位置
	 */
	virtual T x(double t) const = 0;

	/**
	 * @brief t时刻的速度
	 * @param t [in] 时间t
	 * @return 速度
	 */
	virtual T dx(double t) const = 0;

	/**
	 * @brief t时刻的加速度
	 * @param t [in] 时间t
	 * @return 加速度
	 */
	virtual T ddx(double t) const = 0;

	/**
	 * @brief 总时长
	 * @return 规定的总时长
	 */
	virtual double duration() const = 0;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* INTERPOLATOR_H_ */
