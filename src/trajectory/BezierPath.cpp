/*
 * BezierPath.cpp
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */

#include "BezierPath.h"
# include "../math/Integrator.h"
# include <algorithm>

using robot::math::Integrator;

namespace robot {
namespace trajectory {


BezierPath::BezierPath(point &a, point &b, point &c, double dl) : _bIpr(BezierInterpolator::ptr(new BezierInterpolator(a, b, c))), _dl(dl)
{
	init();
}

BezierPath::BezierPath(point &a, point &b, point &c, point &d, double dl) : _bIpr(BezierInterpolator::ptr(new BezierInterpolator(a, b, c, d))), _dl(dl)
{
	init();
}

BezierPath::BezierPath(vector<point>& pointList, double dl) : _bIpr(BezierInterpolator::ptr(new BezierInterpolator(pointList))), _dl(dl)
{
	init();
}

BezierPath::BezierPath(BezierInterpolator::ptr bIpr, double dl): _bIpr(bIpr), _dl(dl)
{
	init();
}

Vector3D<double> BezierPath::x(double l) const
{
	return _bIpr->x(t(l));
}

Vector3D<double> BezierPath::dx(double l) const
{
	return _bIpr->dx(t(l));
}

Vector3D<double> BezierPath::ddx(double l) const
{
	return _bIpr->ddx(t(l));
}

double BezierPath::duration() const
{
	return *(_length.end() - 1);
}

double BezierPath::t(double l) const
{
	auto upper = std::upper_bound(_length.begin(), _length.end(), l);
	int upperIndex = upper - _length.begin();
	if (upperIndex == 0)
		return _t[0]; //0
	int lowerIndex = upperIndex - 1;
	if (upper == _length.end())
	{
		upperIndex -= 1;
		lowerIndex -= 1;
	}
	double l0 = _length[lowerIndex];
	double l1 = _length[upperIndex];
	double t0 = _t[lowerIndex];
	double t1 = _t[upperIndex];
	return (l - l0)*(t1 - t0)/(l1 - l0) + t0;
}

BezierPath::~BezierPath() {
	// TODO Auto-generated destructor stub
}

void BezierPath::init()
{
	double T = _bIpr->duration();
	/**> 第一次采样粗略获得长度, 以确定采样点数 */
	double count = 100;
	Integrator itr;
	double totalLength = itr.integrate(_bIpr, count);
	/**> 根据粗略的长度信息求得采样点数 */
	count = totalLength/_dl + 1;
	count = count < _countMin ? _countMin:count;
	double dt = T/(count - 1);
	/**> 获取时间向量 */
	for (int i=0; i<count - 1; i++)
	{
		_t.push_back(i*dt);
	}
	_t.push_back(T);//此举保证最后一个数值一定是bIpr的总时长(防止计算误差造成不正确后果). 为保证最后一个点的位置的精确度, 这是很有必要的.
	/**> 获取长度向量 */
	_length = itr.integrate(_bIpr, _t);
}

} /* namespace trajectory */
} /* namespace robot */
