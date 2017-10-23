/*
 * RotationInterpolator.h
 *
 *  Created on: Oct 23, 2017
 *      Author: a1994846931931
 */

#ifndef ROTATIONINTERPOLATOR_H_
#define ROTATIONINTERPOLATOR_H_

# include "Interpolator.h"
# include "../math/HTransform3D.h"

namespace robot {
namespace trajectory {

class RotationInterpolator : public Interpolator<Rotation3D<double> >{
public:
	using ptr = std::shared_ptr<RotationInterpolator>;

	RotationInterpolator(Rotation3D<double>& start, Vector3D<double>& n, double rad, double duration=0);

	Rotation3D<double> x(double t) const;

	Rotation3D<double> dx(double t) const;

	Rotation3D<double> ddx(double t) const;

	double duration() const;

	virtual ~RotationInterpolator();
private:
	const Rotation3D<double> _start;

	Vector3D<double> _n;

	const double _rad;

	double _duration;
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* ROTATIONINTERPOLATOR_H_ */
