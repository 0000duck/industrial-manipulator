/**
 * @brief RotationInterpolator.h
 * @date Oct 23, 2017
 * @author a1994846931931
 */

#ifndef ROTATIONINTERPOLATOR_H_
#define ROTATIONINTERPOLATOR_H_

# include "Interpolator.h"
# include "../math/HTransform3D.h"

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */

/**
 * @brief 纯旋转插补器
 *
 * 指定旋转方向和角度的旋转插补器, 相比LinearInterpolator<Rotation3D<>>更具可控性.
 */
class RotationInterpolator : public Interpolator<Rotation3D<double> >{
public:
	using ptr = std::shared_ptr<RotationInterpolator>;

	/**
	 * @brief 构造函数
	 * @param start [in] 开始姿态
	 * @param n [in] 旋转轴
	 * @param rad [in] 旋转角度(rad)
	 * @param duration [in] 时长
	 */
	RotationInterpolator(Rotation3D<double>& start, Vector3D<double>& n, double rad, double duration=0);

	Rotation3D<double> x(double t) const;

	Rotation3D<double> dx(double t) const;

	Rotation3D<double> ddx(double t) const;

	double duration() const;

	virtual ~RotationInterpolator();
private:
	/**> 开始姿态 */
	const Rotation3D<double> _start;

	/**> 旋转轴 */
	Vector3D<double> _n;

	/**> 旋转角度 */
	const double _rad;

	/**> 插补时长 */
	double _duration;
};

/** @} */

} /* namespace trajectory */
} /* namespace robot */

#endif /* ROTATIONINTERPOLATOR_H_ */
