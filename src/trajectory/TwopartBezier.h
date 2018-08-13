/**@brief 两段三次bezier曲线连续过度
 * @file TwopartBezie
 * @date 2017/10/23
 * @author
 */

#ifndef TWOPARTBEZIER_H_
#define TWOPARTBEZIER_H_

# include "Interpolator.h"
# include "../math/Vector3D.h"
# include "../common/printAdvance.h"
# include "../trajectory/BezierInterpolator.h"
# include "../trajectory/SequenceInterpolator.h"
# include "../math/Q.h"

namespace robot {
namespace trajectory {

using robot::math::Vector3D;
using std::vector;
using namespace robot::trajectory;
using namespace robot::math;

class TwopartBezier : public Interpolator<Vector3D<double> >{
public:
	using ptr = std::shared_ptr<TwopartBezier>;
	using point = Vector3D<double>;

	/**@brief 默认构造函数
	 *
	 */
	TwopartBezier() = default;

	/**@brief 曲线生成函数
	 *两段Bezier曲线合成一条过度曲线
	 * @param p1 [in] 点p1
	 * @param p2 [in] 点p2
	 * @param p3 [in] 点p3
	 * @param k [in] 过度大小系数（0～0.5）
	 * @param duration [in] 时长
	 */
	TwopartBezier(const point &p1, const point &p2, const point &p3, double k, double duration);

	/**@brief 位置函数
	 *
	 * @param t
	 * @return t时刻位置
	 */
	robot::math::Vector3D<double> x(double t);

	/**@brief 速度函数
	 *
	 * @param t
	 * @return t时刻速度
	 */
	robot::math::Vector3D<double> dx(double t);

	/**@brief 加速度函数
	 *
	 * @param t
	 * @return t时刻加速度
	 */
	robot::math::Vector3D<double> ddx(double t);

	virtual ~TwopartBezier();

private:
	/** @brief 开始点 */
	Vector3D<double> _p1;

	/** @brief 中间点 */
	Vector3D<double> _p2;

	/** @brief 结束点 */
	Vector3D<double> _p3;

	/** @brief 第一段第一点 */
	Vector3D<double> _b11;

	/** @brief 第一段第二点 */
	Vector3D<double> _b12;

	/** @brief 第二段第一点 */
	Vector3D<double> _b21;

	/** @brief 第二段第二点 */
	Vector3D<double> _b22;

	/** @brief 中间过度点 */
	Vector3D<double> _q;

	/** @brief 插补时长 */
	double _duration;

	/** @brief 插补路径 */
	SequenceInterpolator <Vector3D<double>> ::ptr _part;
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* TWOPARTBEZIER_H_ */
