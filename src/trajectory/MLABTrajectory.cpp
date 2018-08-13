/*
 * MLABTrajectory.cpp
 *
 *  Created on: Sep 26, 2017
 *      Author: a1994846931931
 */

#include "MLABTrajectory.h"

namespace robot {
namespace trajectory {

MLABTrajectory::MLABTrajectory(
		vector<Trajectory::ptr> vtrajectory,
		vector<SequenceInterpolator<double>::ptr > vlt,
		vector<Interpolator<Q>::ptr> vqIpr,
		Trajectory::ptr trajectory,
		SequenceInterpolator<double>::ptr lt,
		Interpolator<Q>::ptr qIpr
		):
		_vtrajectory(vtrajectory), _vlt(vlt), _vqIpr(vqIpr), _trajectory(trajectory), _lt(lt), _qIpr(qIpr)
{
	_size = (int)(vlt.size());
}

Q MLABTrajectory::x(double t) const
{
	return _qIpr->x(t);
}

Q MLABTrajectory::dx(double t) const
{
	return _qIpr->dx(t);
}

Q MLABTrajectory::ddx(double t) const
{
	return _qIpr->ddx(t);
}

double MLABTrajectory::duration() const
{
	return _qIpr->duration();
}


double MLABTrajectory::l(double t) const
{
	return _lt->x(t);
}

double MLABTrajectory::dl(double t) const
{
	return _lt->dx(t);
}

double MLABTrajectory::ddl(double t) const
{
	return _lt->ddx(t);
}

vector<double> MLABTrajectory::getTimeVector() const
{
	vector<double> vt;
	for (int i=0; i<_size; i++)
	{
		vt.push_back(_vlt[i]->duration());
	}
	return vt;
}

vector<double> MLABTrajectory::getLengthVector() const
{
	vector<double> vl;
	for (int i=0; i<_size; i++)
	{
		vl.push_back(_vtrajectory[i]->duration());
	}
	return vl;
}

int MLABTrajectory::getIndexFromTime(double t) const
{
	vector<double> vt = getTimeVector();
	for (int i=0; i<_size; i++)
	{
		if (t >= vt[i])
		{
			t -= vt[i];
		}
		else
			return i;
	}
	return _size - 1;
}

int MLABTrajectory::getIndexFromLength(double l) const
{
	vector<double> vl = getLengthVector();
	for (int i=0; i<_size; i++)
	{
		if (l >= vl[i])
		{
			l -= vl[i];
		}
		else
			return i;
	}
	return _size - 1;
}

} /* namespace trajectory */
} /* namespace robot */
