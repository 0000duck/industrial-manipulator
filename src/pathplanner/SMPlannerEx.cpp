/*
 * SMPlannerEx.cpp
 *
 *  Created on: Sep 25, 2017
 *      Author: a1994846931931
 */

#include "SMPlannerEx.h"
# include <math.h>
# include "../trajectory/PolynomialInterpolator.h"
# include "../trajectory/SequenceInterpolator.h"
# include "../trajectory/LinearInterpolator.h"
# include "../common/printAdvance.h"

using namespace robot::common;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

SMPlannerEx::SMPlannerEx()
{

}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::query(double s, double h, double aMax, double v1, double v2) const
{
	/**> 判断参数合理性 */
	if (s <= 0 || v1 < 0 || v2 < 0)
		throw("错误<SMPlannerEx>: 距离必须为正数!");
	if (v1 < 0 || v2 < 0)
		throw("错误<SMPlannerEx>: 速度必须为非负数!");
	/**> 如果始末速度相同, 则返回线性插补器 */
	if (fabs(v2 - v1) < 1e-10)
	{
		Interpolator<double>::ptr interpolator0(new LinearInterpolator<double>(0, s, 2*s/(v1 + v2)));
		SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
		interpolator->addInterpolator(interpolator0);
		return interpolator;
	}
	/**> 末端速度不为0 */
	if (v2 > 0)
	{
		/**> 三段式加速度 */
		if (fabs(v2 - v1) <= fabs(aMax*aMax/h))
		{
			aMax = sqrt(fabs((v2 - v1)*h));
			return threeLineMotion(s, h, aMax, v1, v2);
		}
		/**> 四段式加速度 */
		else
		{
			return fourLineMotion(s, h, aMax, v1, v2);
		}
	}
	/**> 末端速度为0 */
	else
	{
		/**> 三段式停止规划 */
		if (fabs(v2 - v1) <= fabs(aMax*aMax/h))
		{
			aMax = sqrt(fabs((v2 - v1)*h));
			return threeLineMotion(s, h, aMax, v1);
		}
		/**> 四段式停止规划 */
		else
		{
			return fourLineMotion(s, h, aMax, v1);
		}
	}
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::threeLineMotion(double s, double h, double aMax, double v1, double v2) const
{
	println("过渡三段规划器");

	int sgn = (v2 > v1)? 1:-1;
	h = sgn*fabs(h);
	aMax = sgn*fabs(aMax);

	double t1 = aMax/h;
	double d1 = h*pow(t1, 3)/6.0 + v1*t1;
	double t2 = 2*t1;
	double d2 = (v1 + v2)*t1;
	double t3 = (s - d2)/v2 + t2;
	if (s < d2)
	{
		throw("错误<SMPlannerEx>: 距离不够!");
	}

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(0, v1, 0, h/6.0, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(d1, h*t1*t1/2.0 + v1, h*t1/2.0, -h/6, t2 - t1));
	PolynomialInterpolator2<double>::ptr interpolator3(new PolynomialInterpolator2<double>(d2, v2, 0, t3 - t2));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::fourLineMotion(double s, double h, double aMax, double v1, double v2) const
{
	println("过渡四段规划器");

	double dv = v2 - v1;
	int sgn = (dv > 0)? 1:-1;
	h = sgn*fabs(h);
	aMax = sgn*fabs(aMax);

	double t1 = aMax/h;
	double t2 = (v2 - v1)/aMax;
	double t3 = (v2 - v1)/aMax + aMax/h;
	double d1 = pow(t1, 3)*h/6.0 + v1*t1;
	double d2 = d1 + h*pow(t1, 2)*(t2 - t1)/2.0 + aMax*pow(t2 - t1, 2)/2.0 + v1*(t2 - t1);
	double d3 = dv*dv/(2*aMax) + dv*aMax/(2*h) + v1*t3;
	double t4 = (s - d3)/v2 + t3;
	if (s < d3)
	{
		throw("错误<SMPlannerEx>: 距离不够!");
	}

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(0, v1, 0, h/6.0, t1));
	PolynomialInterpolator2<double>::ptr interpolator2(new PolynomialInterpolator2<double>(d1, t1*t1*h/2 + v1, aMax/2.0, t2 - t1));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(
			d2, (v2 - h*pow(t3 - t2, 2)/2.0), h*(t3 - t2)/2.0, (-h/6.0), t3 - t2));
	PolynomialInterpolator2<double>::ptr interpolator4(new PolynomialInterpolator2<double>(d3, v2, 0, t4 - t3));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::threeLineMotion(double s, double h, double aMax, double v1) const
{
	println("过渡停止三段规划器");

	double v2 = 0;

	int sgn = (v2 > v1)? 1:-1;
	h = sgn*fabs(h);
	aMax = sgn*fabs(aMax);

	double t1 = aMax/h;
	double d1 = h*pow(t1, 3)/6.0 + v1*t1;
	double t2 = 2*t1;
	double d2 = (v1 + v2)*t1;
	if (s < d2)
	{
		throw("错误<SMPlannerEx>: 距离不够!");
	}
	double d0 = (s - d2);
	double t0 = d0/v1;
	d1 += d0;

	PolynomialInterpolator2<double>::ptr interpolator0(new PolynomialInterpolator2<double>(0, v1, 0, t0));
	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(d0, v1, 0, h/6.0, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(d1, h*t1*t1/2.0 + v1, h*t1/2.0, -h/6, t2 - t1));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator0);
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::fourLineMotion(double s, double h, double aMax, double v1) const
{
	println("过渡停止四段规划器");

	double v2 = 0;

	double dv = v2 - v1;
	int sgn = (dv > 0)? 1:-1;
	h = sgn*fabs(h);
	aMax = sgn*fabs(aMax);

	double t1 = aMax/h;
	double t2 = (v2 - v1)/aMax;
	double t3 = (v2 - v1)/aMax + aMax/h;
	double d1 = pow(t1, 3)*h/6.0 + v1*t1;
	double d2 = d1 + h*pow(t1, 2)*(t2 - t1)/2.0 + aMax*pow(t2 - t1, 2)/2.0 + v1*(t2 - t1);
	double d3 = dv*dv/(2*aMax) + dv*aMax/(2*h) + v1*t3;
	if (s < d3)
	{
		throw("错误<SMPlannerEx>: 距离不够!");
	}
	double d0 = s - d3;
	double t0 = d0/v1;
	d1 += d0;
	d2 += d0;

	PolynomialInterpolator2<double>::ptr interpolator0(new PolynomialInterpolator2<double>(0, v1, 0, t0));
	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(d0, v1, 0, h/6.0, t1));
	PolynomialInterpolator2<double>::ptr interpolator2(new PolynomialInterpolator2<double>(d1, t1*t1*h/2 + v1, aMax/2.0, t2 - t1));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(
			d2, (v2 - h*pow(t3 - t2, 2)/2.0), h*(t3 - t2)/2.0, (-h/6.0), t3 - t2));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator0);
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);

	return interpolator;
}

} /* namespace pathplanner */
} /* namespace robot */
