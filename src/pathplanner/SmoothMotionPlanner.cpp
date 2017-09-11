/*
 * SmoothMotionPlanner.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: a1994846931931
 */

# include "SmoothMotionPlanner.h"
# include "math.h"
# include "../trajectory/PolynomialInterpolator.h"
# include "../trajectory/SequenceInterpolator.h"
# include "../common/printAdvance.h"

using namespace robot::common;

namespace robot {
namespace pathplanner {

using namespace robot::trajectory;


robot::trajectory::Interpolator<double>* SmoothMotionPlanner::query(double s, double h, double aMax, double vMax, double start)
{
	if (h <= 0 || aMax <= 0 || vMax <= 0)
		throw ("错误: 参数必须为正! < 来自 SmoothMotionPlanner::query");
	if (s == 0)
	{
		Interpolator<double>* interpolator = new FixedInterpolator<double>(start, 0.001);
		_interpolatorList.push_back(interpolator);
		return interpolator;
	}
	if (vMax <= (aMax*aMax/h))
	{
		aMax = sqrt(vMax*h);
		if (fabs(s) > 2*s2(h, aMax))
			return this->fiveLineMotion(s, h, aMax, vMax, start);
		else
		{
			aMax = pow((fabs(s)*h*h/2.0), 1.0/3.0);
			vMax = aMax*aMax/h;
			return this->fourLineMotion(s, h, aMax, vMax, start);
		}
	}
	else
	{
		if (fabs(s) > 2*s1(h, vMax, aMax))
			return this->sevenLineMotion(s, h, aMax, vMax, start);
		else
		{
			if (fabs(s) > 2*s2(h, aMax))
			{
				vMax = (-aMax*aMax + sqrt(pow(aMax, 4) + 4*h*h*aMax*fabs(s)))/(2*h);
				return sixLineMotion(s, h, aMax, vMax, start);
			}
			else
			{
				aMax = pow((fabs(s)*h*h/2.0), 1.0/3.0);
				vMax = aMax*aMax/h;
				return this->fourLineMotion(s, h, aMax, vMax, start);
			}
		}
	}
}

robot::trajectory::Interpolator<double>* SmoothMotionPlanner::fourLineMotion(double s, double h, double aMax, double vMax, double start)
{
	println("四段规划器");
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double t1 = aMax/h;
	double t3 = t1;
	double t5 = t3;
	double t7 = t1;
	double d1 = pow(aMax, 3)/(6*h*h);
	double d2 = d1;
	double d3 = vMax*aMax/h;
	double d4 = d3;

	PolynomialInterpolator3<double>* interpolator1 = new PolynomialInterpolator3<double>(start, 0, 0, sign*(h/6), t1);
	PolynomialInterpolator3<double>* interpolator2 = new PolynomialInterpolator3<double>(
			sign*(d2) + start, sign*(vMax - pow(t3, 2)*h/2), sign*(h*t3/2), sign*(-h/6), t3);
	PolynomialInterpolator3<double>* interpolator3 = new PolynomialInterpolator3<double>(sign*(d4) + start, sign*(vMax), 0, sign*(-h/6), t5);
	PolynomialInterpolator3<double>* interpolator4 = new PolynomialInterpolator3<double>(
			sign*(s - pow(t7, 3)*h/6) + start, sign*(pow(t7, 2)*h/2), sign*(-t7*h/2), sign*(h/6), t7);

	SequenceInterpolator<double>* interpolator = new SequenceInterpolator<double>();
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);

	_interpolatorList.push_back(interpolator1);
	_interpolatorList.push_back(interpolator2);
	_interpolatorList.push_back(interpolator3);
	_interpolatorList.push_back(interpolator4);
	_interpolatorList.push_back(interpolator);

	return interpolator;
}

robot::trajectory::Interpolator<double>* SmoothMotionPlanner::fiveLineMotion(double s, double h, double aMax, double vMax, double start)
{
	println("五段规划器");
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double t1 = aMax/h;
	double t3 = t1;
	double t4 = s/vMax - 2*sqrt(vMax/h);
	double t5 = t3;
	double t7 = t1;
	double d1 = pow(aMax, 3)/(6*h*h);
	double d2 = d1;
	double d3 = vMax*aMax/h;
	double d4 = s - d3;

	PolynomialInterpolator3<double>* interpolator1 = new PolynomialInterpolator3<double>(start, 0, 0, sign*(h/6), t1);
	PolynomialInterpolator3<double>* interpolator2 = new PolynomialInterpolator3<double>(
			sign*d2 + start, sign*(vMax - h*pow(t3, 2)/2), sign*h*t3/2, sign*(-h/6.0), t3);
	PolynomialInterpolator3<double>* interpolator3 = new PolynomialInterpolator3<double>(sign*d3 + start, sign*vMax, 0, 0, t4);
	PolynomialInterpolator3<double>* interpolator4 = new PolynomialInterpolator3<double>(sign*d4 + start, sign*vMax, 0, sign*(-h/6.0), t5);
	PolynomialInterpolator3<double>* interpolator5 = new PolynomialInterpolator3<double>(
			sign*(-pow(t7, 3)*h/6.0 + s) + start, sign*(pow(t7, 2)*h/2.0), sign*(-h*t7/2.0), sign*(h/6.0), t7);

	SequenceInterpolator<double>* interpolator = new SequenceInterpolator<double>();
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);

	_interpolatorList.push_back(interpolator1);
	_interpolatorList.push_back(interpolator2);
	_interpolatorList.push_back(interpolator3);
	_interpolatorList.push_back(interpolator4);
	_interpolatorList.push_back(interpolator5);
	_interpolatorList.push_back(interpolator);

	return interpolator;
}

robot::trajectory::Interpolator<double>* SmoothMotionPlanner::sixLineMotion(double s, double h, double aMax, double vMax, double start)
{
	println("六段规划器");
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double s1 = this->s1(h, vMax, aMax);
	double t1 = aMax/h;
	double t2 = vMax/aMax - aMax/h;
	double t3 = t1;
	double t5 = t3;
	double t6 = t2;
	double t7 = t1;
	double d1 = pow(aMax, 3)/(6*pow(h, 2));
	double d2 = d1 + h*pow(t1, 2)*t2/2.0 + aMax*pow(t2, 2)/2.0;
	double d3 = s1;
	double d4 = s - d3;
	double d5 = s - d2;

	PolynomialInterpolator3<double>* interpolator1 = new PolynomialInterpolator3<double>(start, 0, 0, sign*h/6, t1);
	PolynomialInterpolator3<double>* interpolator2 = new PolynomialInterpolator3<double>(
			sign*d1 + start, sign*h*pow(t1, 2)/2, sign*aMax/2.0, 0, t2);
	PolynomialInterpolator3<double>* interpolator3 = new PolynomialInterpolator3<double>(
			sign*d2 + start, sign*(vMax - h*t3*t3/2.0), sign*h*t3/2.0, sign*(-h/6.0), t3);
	PolynomialInterpolator3<double>* interpolator4 = new PolynomialInterpolator3<double>(sign*d4 + start, sign*vMax, 0, sign*(-h/6.0), t5);
	PolynomialInterpolator3<double>* interpolator5 = new PolynomialInterpolator3<double>(
			sign*d5 + start, sign*(vMax - h*t5*t5/2), sign*(-aMax/2), 0, t6);
	PolynomialInterpolator3<double>* interpolator6 = new PolynomialInterpolator3<double>(
			sign*(s - h*pow(t7, 3)/6.0) + start, sign*h*t7*t7/2.0, sign*(-h*t7/2.0), sign*h/6.0, t7);

	SequenceInterpolator<double>* interpolator = new SequenceInterpolator<double>();
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);
	interpolator->addInterpolator(interpolator6);

	_interpolatorList.push_back(interpolator1);
	_interpolatorList.push_back(interpolator2);
	_interpolatorList.push_back(interpolator3);
	_interpolatorList.push_back(interpolator4);
	_interpolatorList.push_back(interpolator5);
	_interpolatorList.push_back(interpolator6);
	_interpolatorList.push_back(interpolator);

	return interpolator;
}

robot::trajectory::Interpolator<double>* SmoothMotionPlanner::sevenLineMotion(double s, double h, double aMax, double vMax, double start)
{
	println("七段规划器");
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double s1 = this->s1(h, vMax, aMax);
	double t1 = aMax/h;
	double t2 = vMax/aMax - aMax/h;
	double t3 = t1;
	double t4 = (s - 2*s1)/vMax;
	double t5 = t3;
	double t6 = t2;
	double t7 = t1;
	double d1 = pow(aMax, 3)/(6*pow(h, 2));
	double d2 = d1 + h*pow(t1, 2)*t2/2.0 + aMax*pow(t2, 2)/2.0;
	double d3 = s1;
	double d4 = s - d3;
	double d5 = s - d2;

	PolynomialInterpolator3<double>* interpolator1 = new PolynomialInterpolator3<double>(start, 0, 0, sign*h/6, t1);
	PolynomialInterpolator3<double>* interpolator2 = new PolynomialInterpolator3<double>(
			sign*d1 + start, sign*h*pow(t1, 2)/2, sign*aMax/2.0, 0, t2);
	PolynomialInterpolator3<double>* interpolator3 = new PolynomialInterpolator3<double>(
			sign*d2 + start, sign*(vMax - h*t3*t3/2.0), sign*h*t3/2.0, sign*(-h/6.0), t3);
	PolynomialInterpolator3<double>* interpolator4 = new PolynomialInterpolator3<double>(sign*d3 + start, sign*vMax, 0, 0, t4);
	PolynomialInterpolator3<double>* interpolator5 = new PolynomialInterpolator3<double>(sign*d4 + start, sign*vMax, 0, sign*(-h/6.0), t5);
	PolynomialInterpolator3<double>* interpolator6 = new PolynomialInterpolator3<double>(
			sign*d5 + start, sign*(vMax - h*t5*t5/2), sign*(-aMax/2), 0, t6);
	PolynomialInterpolator3<double>* interpolator7 = new PolynomialInterpolator3<double>(
			sign*(s - h*pow(t7, 3)/6.0) + start, sign*h*t7*t7/2.0, sign*(-h*t7/2.0), sign*h/6.0, t7);

	SequenceInterpolator<double>* interpolator = new SequenceInterpolator<double>();
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);
	interpolator->addInterpolator(interpolator6);
	interpolator->addInterpolator(interpolator7);

	_interpolatorList.push_back(interpolator1);
	_interpolatorList.push_back(interpolator2);
	_interpolatorList.push_back(interpolator3);
	_interpolatorList.push_back(interpolator4);
	_interpolatorList.push_back(interpolator5);
	_interpolatorList.push_back(interpolator6);
	_interpolatorList.push_back(interpolator7);
	_interpolatorList.push_back(interpolator);

	return interpolator;
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
