/*
 * CircularTrajectory.cpp
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */

#include "CircularPath.h"

namespace robot {
namespace trajectory {

CircularPath::CircularPath(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
		std::shared_ptr<robot::ik::IKSolver> iksolver,
		robot::model::Config config,
		SequenceInterpolator<double>::ptr lt,
		Trajectory::ptr trajectory)
:_qIpr(new ikInterpolator(origin, iksolver, config)), _lt(lt), _trajectory(trajectory), _pathSize(trajectory->duration()/0.01 + 1)
{
	_lengthPath.reserve(_pathSize);
}

Q CircularPath::x(double t) const
{
	return _qIpr->x(t);
}

Q CircularPath::dx(double t) const
{
	return _qIpr->dx(t);
}

Q CircularPath::ddx(double t) const
{
	return _qIpr->ddx(t);
}

double CircularPath::l(double t) const
{
	return _lt->x(t);
}

double CircularPath::dl(double t) const
{
	return _lt->dx(t);
}

double CircularPath::ddl(double t) const
{
	return _lt->ddx(t);
}

double CircularPath::duration() const
{
	return _lt->duration();
}

} /* namespace trajectory */
} /* namespace robot */
