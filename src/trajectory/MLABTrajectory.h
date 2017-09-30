/*
 * MLABTrajectory.h
 *
 *  Created on: Sep 26, 2017
 *      Author: a1994846931931
 */

#ifndef MLABTRAJECTORY_H_
#define MLABTRAJECTORY_H_

#include "Interpolator.h"
# include "../math/Q.h"
# include <memory>
# include "CircularInterpolator.h"
# include "LinearInterpolator.h"
# include "ConvertedInterpolator.h"
# include "Trajectory.h"

using robot::math::Q;
using std::vector;

namespace robot {
namespace trajectory {

class MLABTrajectory: public Interpolator<Q> {
public:
	using ptr = std::shared_ptr<MLABTrajectory>;

	MLABTrajectory(
			vector<CircularInterpolator<Vector3D<double> >::ptr> arcPosIpr,
			vector<LinearInterpolator<Vector3D<double> >::ptr> linePosIpr,
			vector<LinearInterpolator<Rotation3D<double> >::ptr> arcRotIpr,
			vector<LinearInterpolator<Rotation3D<double> >::ptr> lineRotIpr,
			vector<double> length,
			vector<Interpolator<Q>::ptr> qIpr,
			vector<Trajectory::ptr> trajectoryIpr,
			vector<Interpolator<double>::ptr > lt);

	Q x(double t) const;
	Q dx(double t) const;
	Q ddx(double t) const;
	double duration() const;
	vector<Interpolator<Vector3D<double> >::ptr> getPosIpr() const;
	virtual ~MLABTrajectory(){}
private:
	/** @brief 线段段数 n */
	int _size;

	/** 以下均以每一段的路径长度作为索引 */
	/** @brief 圆弧位置插补器 n-1 */
	vector<CircularInterpolator<Vector3D<double> >::ptr> _arcPosIpr;
	/** @brief 直线位置插补器 n */
	vector<LinearInterpolator<Vector3D<double> >::ptr> _linePosIpr;
	/** @brief 圆弧姿态插补器 n-1 */
	vector<LinearInterpolator<Rotation3D<double> >::ptr> _arcRotIpr;
	/** @brief 直线姿态插补器 n */
	vector<LinearInterpolator<Rotation3D<double> >::ptr> _lineRotIpr;

	/** @brief 各段的长度 2n-1 */
	vector<double> _length;
	/** @brief 各段的ikInterpolator 长度为索引 */
	vector<Trajectory::ptr> _trajectoryIpr;
	/** @brief 各段的ikInterpolator 2n-1 时间为索引*/
	vector<Interpolator<Q>::ptr > _qIpr;

	/** @brief l(t) 2n-1 */
	vector<Interpolator<double>::ptr > _lt;
	/** @brief t(l) 2n-1 */
//	Interpolator<double>::ptr _tl;

};

} /* namespace trajectory */
} /* namespace robot */

#endif /* MLABTRAJECTORY_H_ */
