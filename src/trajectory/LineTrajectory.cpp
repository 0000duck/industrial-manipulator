/*
 * LineTrajectory.cpp
 *
 *  Created on: Sep 20, 2017
 *      Author: a1994846931931
 */

#include "LineTrajectory.h"
# include "time.h"
# include "../math/Integrator.h"

namespace robot {
namespace trajectory {

//LineTrajectory::LineTrajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
//		std::shared_ptr<robot::ik::IKSolver> iksolver,
//		robot::model::Config config,
//		LinearCompositeInterpolator<double>::ptr mappedlt,
//		LinearCompositeInterpolator<double>::ptr mappedtt)
//:ikInterpolator(origin, iksolver, config), _lt(mappedlt), _tt(mappedtt), _pathSize(1000)
//{
//	_lengthPath.reserve(_pathSize);
//}

LineTrajectory::LineTrajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
		std::shared_ptr<robot::ik::IKSolver> iksolver,
		robot::model::Config config,
		SequenceInterpolator<double>::ptr lt,
		Trajectory::ptr trajectory)
:_qIpr(new ikInterpolator(origin, iksolver, config)), _trajectory(trajectory), _lt(lt), _pathSize(trajectory->duration()/0.01 + 1)
{
	_lengthPath.reserve(_pathSize);
}

Q LineTrajectory::x(double t) const
{
	return _qIpr->x(t);
}

Q LineTrajectory::dx(double t) const
{
	return _qIpr->dx(t);
}

Q LineTrajectory::ddx(double t) const
{
	return _qIpr->ddx(t);
}

double LineTrajectory::l(double t) const
{
	return _lt->x(t);
}

double LineTrajectory::dl(double t) const
{
	return _lt->dx(t);
}

double LineTrajectory::ddl(double t) const
{
	return _lt->ddx(t);
}

double LineTrajectory::duration() const
{
	return _lt->duration();
}

double LineTrajectory::timeAt(double length) const
{
	if ((int)_lengthPath.size() <= 1)
		throw(std::string("错误<LineInterpolator>: 尚未进行过路径长度分析, 无法获取目标位置的时间!"));
	double t;
	const int size = _lengthPath.size();
	if (length <= 0)
		t = 0;
	else if (length >= _lengthPath[size - 1].second)
	{
		t = _lengthPath[size - 1].first;
	}
	else
	{
		int a = 0;
		int b = size - 1;
		int c;
		while((b - a) > 1)
		{
			c = (int)((a + b)/2);
			if (length < _lengthPath[c].second)
				b = c;
			else
				a = c;
		}
		double ta = _lengthPath[a].first;
		double tb = _lengthPath[b].first;
		double ya = _lengthPath[a].second;
		double yb = _lengthPath[b].second;
		if (ya == yb)
			t = ta;
		else
			t = (tb - ta)*(length - ya)/(yb - ya) + ta;
	}
	return t;
}

void LineTrajectory::doLengthAnalysis()
{
	clock_t start = clock();
	if ((int)_lengthPath.size() > 0)
	{
		println("警告<LineInterpolator>: 已经做过一次长度分析");
	}
	/**> 做距离采样 */
	double T = this->duration();
	double dt = T/(_pathSize - 1);
	for (double t = 0; t< T; t+=dt)
	{
		/** 时刻, 距离 **/
		_lengthPath.push_back(std::pair<double, double>(t, _lt->x(t)));
	}
	_lengthPath.push_back(std::pair<double, double>(T, _lt->x(T)));
	clock_t end = clock();
	cout << "<LineInterpolator>: 完成直线路径长度分析, 用时: " << end - start << endl;
}

Trajectory::ptr LineTrajectory::getTrajectory() const
{
	return _trajectory;
}

} /* namespace trajectory */
} /* namespace robot */
