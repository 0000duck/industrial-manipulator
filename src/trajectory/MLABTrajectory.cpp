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
		vector<CircularInterpolator<Vector3D<double> >::ptr> arcPosIpr,
		vector<LinearInterpolator<Vector3D<double> >::ptr> linePosIpr,
		vector<LinearInterpolator<Rotation3D<double> >::ptr> arcRotIpr,
		vector<LinearInterpolator<Rotation3D<double> >::ptr> lineRotIpr,
		vector<double> length,
		vector<Interpolator<Q>::ptr> qIpr,
		vector<Trajectory::ptr> trajectoryIpr,
		vector<SequenceInterpolator<double>::ptr > lt,
		std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
		std::shared_ptr<robot::ik::IKSolver> iksolver,
		robot::model::Config config):
			_varcPosIpr ( arcPosIpr),
			_vlinePosIpr ( linePosIpr),
			_varcRotIpr ( arcRotIpr),
			_vlineRotIpr ( lineRotIpr),
			_vlength ( length),
			_vtrajectoryIpr( trajectoryIpr),
			_vqIpr ( qIpr),
			_vlt ( lt),
			_lt( new SequenceInterpolator<double>()),
			_trajectory( new Trajectory(origin, iksolver, config))
{
	_size = (int)(linePosIpr.size() + arcRotIpr.size());
	_vt.push_back(0);
	_vl.push_back(0);
	for (int i=0; i<(int)_vlt.size() - 1; i++)
	{
		_vt.push_back(_vt[i] + _vlt[i]->duration());
		_vl.push_back(_vl[i] + _vlt[i]->x(_vt[i + 1] - _vt[i]));
	}
	for (int i=0; i<(int)_vt.size(); i++)
	{
		cout << "t" << i << " = " << _vt[i] << endl;
	}
	for (int i=0; i<(int)_vlt.size(); i++)
	{
		_lt->appendInterpolator(_vlt[i]);
	}
}

Q MLABTrajectory::x(double t) const
{
	int i=(int)_vt.size() - 1;
	for (; i >= 0; i--)
	{
		if (t >= _vt[i])
			break;
	}
	////////////************////////////
//	println("MLABTrajec...");
//	cout << "i = " << i << endl;
	return _vqIpr[i]->x(t - _vt[i]);
}

Q MLABTrajectory::dx(double t) const
{
	int i=(int)_vt.size() - 1;
	for (; i >= 0; i--)
	{
		if (t >= _vt[i])
			break;
	}
	return _vqIpr[i]->dx(t - _vt[i]);
}

Q MLABTrajectory::ddx(double t) const
{
	int i=(int)_vt.size() - 1;
	for (; i >= 0; i--)
	{
		if (t >= _vt[i])
			break;
	}
	return _vqIpr[i]->ddx(t - _vt[i]);
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

double MLABTrajectory::duration() const
{
	return _lt->duration();
}

vector<Interpolator<Vector3D<double> >::ptr> MLABTrajectory::getPosIpr() const
{
	vector<Interpolator<Vector3D<double> >::ptr> posIpr;

	bool indexOnLine = true;
	for (int i=0; i<(int)_size; i++)
	{
		if (indexOnLine)
		{
			posIpr.push_back(_vlinePosIpr[i/2]);
			indexOnLine = !indexOnLine;
		}
		else
		{
			posIpr.push_back(_varcPosIpr[(i - 1)/2]);
			indexOnLine = !indexOnLine;
		}
	}
	return posIpr;
}

} /* namespace trajectory */
} /* namespace robot */
