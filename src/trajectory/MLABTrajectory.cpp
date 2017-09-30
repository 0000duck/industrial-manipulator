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
		vector<Interpolator<double>::ptr > lt):
	_arcPosIpr ( arcPosIpr),
	_linePosIpr ( linePosIpr),
	_arcRotIpr ( arcRotIpr),
	_lineRotIpr ( lineRotIpr),
	_length ( length),
	_trajectoryIpr( trajectoryIpr),
	_qIpr ( qIpr),
	_lt ( lt)
{
	_size = (int)(linePosIpr.size() + arcRotIpr.size());
}

Q MLABTrajectory::x(double t) const
{
	double maxDuration = 0;
	int i=0;
	for (; i<(int)_lt.size(); i++)
	{
		maxDuration += _lt[i]->duration();
		if (t < maxDuration)
			break;
	}
	if (i == (int)_lt.size())
		i--;
	////////////************////////////
//	println("MLABTrajec...");
//	cout << "qiIpr size is: " << _qIpr.size() << endl;
//	cout << "i = " << i << endl;
	return _qIpr[i]->x(t - maxDuration + _lt[i]->duration());
}

Q MLABTrajectory::dx(double t) const
{
	double maxDuration = 0;
	int i=0;
	for (; i<(int)_lt.size(); i++)
	{
		maxDuration += _lt[i]->duration();
		if (t < maxDuration)
			break;
	}
	if (i == (int)_lt.size())
		i--;
	return _qIpr[i]->dx(t);
}

Q MLABTrajectory::ddx(double t) const
{
	double maxDuration = 0;
	int i=0;
	for (; i<(int)_lt.size(); i++)
	{
		maxDuration += _lt[i]->duration();
		if (t < maxDuration)
			break;
	}
	if (i == (int)_lt.size())
		i--;
	return _qIpr[i]->ddx(t);
}

double MLABTrajectory::duration() const
{
	double duration = 0;
	for (int i=0; i<(int)_lt.size(); i++)
	{
		duration += _lt[i]->duration();
	}
	return duration;
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
