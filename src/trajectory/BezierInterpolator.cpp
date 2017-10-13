/*
 * BezierInterpolator.cpp
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */

#include "BezierInterpolator.h"

namespace robot {
namespace trajectory {

using point = Vector3D<double>;

BezierInterpolator::BezierInterpolator(point &a, point &b, point &c)
{
	_vpoint.push_back(a);
	_vpoint.push_back(b);
	_vpoint.push_back(c);
	_size = 3;
}

BezierInterpolator::BezierInterpolator(point &a, point &b, point &c, point &d)
{
	_vpoint.push_back(a);
	_vpoint.push_back(b);
	_vpoint.push_back(c);
	_vpoint.push_back(d);
	_size = 4;
}

BezierInterpolator::BezierInterpolator(vector<point>& pointList)
{
	_size = (int)pointList.size();
	if (_size < 2)
	{
		throw (string("错误<BezierInterpolator>: 无法构造目标点数小于2的贝赛尔曲线!"));
	}
	for (int i=0; i<_size; i++)
	{
		_vpoint.push_back(pointList[i]);
	}
}

Vector3D<double> BezierInterpolator::x(double s) const
{
	if (_size == 3)
		return x3(s/_duration, _vpoint);
	if (_size == 4)
		return x4(s/_duration, _vpoint);
	return xn(s/_duration, _vpoint);
}

Vector3D<double> BezierInterpolator::dx(double s) const
{
	if (_size == 3)
		return dx3(s/_duration, _vpoint);
	if (_size == 4)
		return dx4(s/_duration, _vpoint);
	return dxn(s/_duration, _vpoint);
}

Vector3D<double> BezierInterpolator::ddx(double s) const
{
	if (_size == 3)
		return ddx3(s/_duration, _vpoint);
	if (_size == 4)
		return ddx4(s/_duration, _vpoint);
	return ddxn(s/_duration, _vpoint);
}

double BezierInterpolator::duration() const
{
	return _duration;
}

BezierInterpolator::~BezierInterpolator() {
	// TODO Auto-generated destructor stub
}

Vector3D<double> BezierInterpolator::x3(double k, const vector<point>& pointList) const
{
	double j = 1.0 - k;
	return pointList[0]*(j*j) + pointList[1]*(2.0*j*k)+ pointList[2]*(k*k);
}

Vector3D<double> BezierInterpolator::dx3(double k, const vector<point>& pointList) const
{
	double j = 1.0 - k;
	return pointList[0]*(-2.0*j) + pointList[1]*(2.0 - 4.0*k)+ pointList[2]*(2.0*k);
}

Vector3D<double> BezierInterpolator::ddx3(double k, const vector<point>& pointList) const
{
	return pointList[0]*(2.0) + pointList[1]*(-4.0)+ pointList[2]*(2.0);
}

Vector3D<double> BezierInterpolator::x4(double k, const vector<point>& pointList) const
{
	double j = 1.0 - k;
	return pointList[0]*(j*j*j) + pointList[1]*(3.0*j*j*k)+ pointList[2]*(3.0*j*k*k) +  pointList[2]*(k*k*k);
}

Vector3D<double> BezierInterpolator::dx4(double k, const vector<point>& pointList) const
{
	double j = 1.0 - k;
	return pointList[0]*(-3.0*j*j) + pointList[1]*(3.0 - 12.0*k + 9.0*k*k)+ pointList[2]*(6.0*k - 9.0*k*k) +  pointList[2]*(3.0*k*k);
}

Vector3D<double> BezierInterpolator::ddx4(double k, const vector<point>& pointList) const
{
	double j = 1.0 - k;
	return pointList[0]*(6.0*j) + pointList[1]*(-12.0 + 18.0*k)+ pointList[2]*(6.0 - 18.0*k) +  pointList[2]*(6.0*k);
}

Vector3D<double> BezierInterpolator::xn(double k, const vector<point>& pointList) const
{
	int size = (int)pointList.size();
	if (size == 1)
		return pointList[0];
	else
	{
		vector<point> newList;
		for (int i=0; i<size - 1; i++)
			newList.push_back(interpolate(pointList[i], pointList[i + 1], k));
		return xn(k, newList);
	}
}

Vector3D<double> BezierInterpolator::dxn(double k, const vector<point>& pointList) const
{
	double precision = 0.00000001;
	return (xn(k + precision, pointList) - xn(k, pointList))/precision;
}

Vector3D<double> BezierInterpolator::ddxn(double k, const vector<point>& pointList) const
{
	double precision = 0.00000001;
	return (xn(k + 2.0*precision, pointList) - xn(k + precision, pointList)*2.0 + xn(k, pointList))/(precision*precision);
}

point BezierInterpolator::interpolate(const point &p1, const point &p2, const double &k)
{
	return p1*(1.0 -k) + p2*k;
}

} /* namespace trajectory */
} /* namespace robot */
