/*
 * CircularTrajectory.cpp
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */

#include "CircularTrajectory.h"

namespace robot {
namespace trajectory {

CircularTrajectory::CircularTrajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
		std::shared_ptr<robot::ik::IKSolver> iksolver,
		robot::model::Config config,
		SequenceInterpolator<double>::ptr lt,
		Trajectory::ptr trajectory)
:_qIpr(new ikInterpolator(origin, iksolver, config)), _lt(lt), _trajectory(trajectory)
{
}

Q CircularTrajectory::x(double t) const
{
	return _qIpr->x(t);
}

Q CircularTrajectory::dx(double t) const
{
	return _qIpr->dx(t);
}

Q CircularTrajectory::ddx(double t) const
{
	return _qIpr->ddx(t);
}

double CircularTrajectory::l(double t) const
{
	return _lt->x(t);
}

double CircularTrajectory::dl(double t) const
{
	return _lt->dx(t);
}

double CircularTrajectory::ddl(double t) const
{
	return _lt->ddx(t);
}

double CircularTrajectory::duration() const
{
	return _lt->duration();
}

} /* namespace trajectory */
} /* namespace robot */
