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
			Trajectory(origin, iksolver, config),
			_arcPosIpr ( arcPosIpr),
			_linePosIpr ( linePosIpr),
			_arcRotIpr ( arcRotIpr),
			_lineRotIpr ( lineRotIpr),
			_length ( length),
			_trajectoryIpr( trajectoryIpr),
			_qIpr ( qIpr),
			_vlt ( lt)
{
	_size = (int)(linePosIpr.size() + arcRotIpr.size());
	_t.push_back(0);
	_l.push_back(0);
	for (int i=0; i<(int)_vlt.size() - 1; i++)
	{
		_t.push_back(_t[i] + _vlt[i]->duration());
		_l.push_back(_l[i] + _vlt[i]->x(_t[i + 1] - _t[i]));
	}
	for (int i=0; i<(int)_t.size(); i++)
	{
		cout << "t" << i << " = " << _t[i] << endl;
	}
	for (int i=0; i<(int)_vlt.size(); i++)
	{
		_lt->appendInterpolator(_vlt[i]);
	}
}

Q MLABTrajectory::x(double t) const
{
	int i=(int)_t.size() - 1;
	for (; i >= 0; i--)
	{
		if (t >= _t[i])
			break;
	}
	////////////************////////////
//	println("MLABTrajec...");
//	cout << "i = " << i << endl;
	return _qIpr[i]->x(t - _t[i]);
}

Q MLABTrajectory::dx(double t) const
{
	int i=(int)_t.size() - 1;
	for (; i >= 0; i--)
	{
		if (t >= _t[i])
			break;
	}
	return _qIpr[i]->dx(t - _t[i]);
}

Q MLABTrajectory::ddx(double t) const
{
	int i=(int)_t.size() - 1;
	for (; i >= 0; i--)
	{
		if (t >= _t[i])
			break;
	}
	return _qIpr[i]->ddx(t - _t[i]);
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
			posIpr.push_back(_linePosIpr[i/2]);
			indexOnLine = !indexOnLine;
		}
		else
		{
			posIpr.push_back(_arcPosIpr[(i - 1)/2]);
			indexOnLine = !indexOnLine;
		}
	}
	return posIpr;
}

} /* namespace trajectory */
} /* namespace robot */
