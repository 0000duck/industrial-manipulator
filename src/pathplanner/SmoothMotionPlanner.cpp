/*
 * SmoothMotionPlanner.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: a1994846931931
 */

# include "SmoothMotionPlanner.h"
# include "math.h"
# include "../trajectory/PolynomialInterpolator.h"

namespace robot {
namespace pathplanner {

using namespace robot::trajectory;

SmoothMotionPlanner::SmoothMotionPlanner()
{
}

static robot::trajectory::Interpolator<double>* SmoothMotionPlanner::query(double s, double h, double aMax, double vMax)
{
	if (s <= 0 || h <= 0 || aMax <= 0 || vMax <= 0)
		throw ("错误: 参数必须为正! < 来自 SmoothMotionPlanner::query");
	if (vMax <= (aMax*aMax/h))
	{
		aMax = sqrt(vMax*h);
		if (s > 2*s2(h, aMax))
			return this->fiveLineMotion(s, h, aMax, vMax);
		else
		{
			aMax = pow((s*h*h/2.0), 1.0/3.0);
			vMax = aMax*aMax/h;
			return this->fourLineMotion(s, h, aMax, vMax);
		}
	}
	else
	{
		if (s > 2*s1(h, vMax, aMax))
			return this->sevenLineMotion(s, h, aMax, vMax);
		else
		{
			if (s > 2*s2(h, aMax))
			{
				vMax = (-aMax/h + sqrt(pow(pow(aMax, 2)/h, 2)))/2;
				return sixLineMotion(s, h, aMax, vMax);
			}
			else
			{
				aMax = pow((s*h*h/2.0), 1.0/3.0);
				vMax = aMax*aMax/h;
				return this->fourLineMotion(s, h, aMax, vMax);
			}
		}
	}
}

robot::trajectory::Interpolator<double>* SmoothMotionPlanner::fourLineMotion(double s, double h, double aMax, double vMax)
{
	double
}

robot::trajectory::Interpolator<double>* SmoothMotionPlanner::fiveLineMotion(double s, double h, double aMax, double vMax)
{
	double t1 = aMax/h;
	double t2 = 0;
	double t3 = t1;
	double t4 = s/vMax - 2*sqrt(vMax/h);
	double t5 = t3;
	double t6 = 0;
	double t7 = t1;

	 Interpolator<double>* interpolator1 = new PolynomialInterpolator3<double>(0, 0, 0, h/6, t1);
}

robot::trajectory::Interpolator<double>* SmoothMotionPlanner::sixLineMotion(double s, double h, double aMax, double vMax)
{

}

robot::trajectory::Interpolator<double>* SmoothMotionPlanner::sevenLineMotion(double s, double h, double aMax, double vMax)
{

}

SmoothMotionPlanner::~SmoothMotionPlanner()
{
	for (int i=0; i<(int)_interpolatorList.size(); i++)
		delete _interpolatorList[i];
}

double SmoothMotionPlanner::s1(double h, double vMax, double aMax)
{
	return (h*vMax*vMax + vMax*aMax*aMax)/(2*h*aMax);
}

double SmoothMotionPlanner::s2(double h, double aMax)
{
	return (aMax*aMax*aMax)/(h*h);
}

} /* namespace pathplanner */
} /* namespace robot */
