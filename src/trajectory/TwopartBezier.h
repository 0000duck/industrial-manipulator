/*
 * TwopartBezier.h
 *
 *  Created on: Oct 13, 2017
 *      Author: zrf
 */

#ifndef TWOPARTBEZIER_H_
#define TWOPARTBEZIER_H_

# include "Interpolator.h"
# include "../math/Vector3D.h"
# include "../common/printAdvance.h"
# include "../trajectory/BezierInterpolator.h"
# include "../trajectory/SequenceInterpolator.h"

namespace robot {
namespace trajectory {

using robot::math::Vector3D;
using std::vector;

class TwopartBezier : public Interpolator<Vector3D<double> >{
public:
	using ptr = std::shared_ptr<TwopartBezier>;
	using point = Vector3D<double>;
	//using namespace robot::trajectory;

	TwopartBezier();

	TwopartBezier(const point &p1, const point &p2, const point &p3, double k, double duration);

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
