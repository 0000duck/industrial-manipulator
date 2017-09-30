/*
 * SMPlannerEx.cpp
 *
 *  Created on: Sep 25, 2017
 *      Author: a1994846931931
 */

#include "SMPlannerEx.h"
# include <math.h>
# include "SmoothMotionPlanner.h"
# include "../trajectory/PolynomialInterpolator.h"
# include "../trajectory/SequenceInterpolator.h"
# include "../trajectory/LinearInterpolator.h"
# include "../common/printAdvance.h"
# include "../common/common.h"

using namespace robot::common;
using namespace robot::trajectory;

namespace robot {
namespace pathplanner {

SMPlannerEx::SMPlannerEx()
{

}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::query(double start, double s, double h, double aMax, double v1, double v2, bool stop) const
{
	if (stop && (fixZero(v2) != 0))
		return query_stop(start, s, h, aMax, v1, v2);
	/**> 判断参数合理性 */
	if (s <= 0)
		throw("错误<SMPlannerEx>: 距离必须为正数!");
	if (fixZero(v1) < 0 || fixZero(v2) < 0)
		throw("错误<SMPlannerEx>: 速度必须为非负数!");
	if	(fixZero(v1) == 0 && fixZero(v2) == 0)
		throw("错误<SMPlannerEx>: 速度不能同时为0");
	/**> 如果始末速度相同, 则返回线性插补器 */
	if (fabs(v2 - v1) < 1e-10)
	{
		cout << "线性规划" << endl;
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
			return threeLineMotion(start, s, h, aMax, v1, v2);
		}
		/**> 四段式加速度 */
		else
		{
			return fourLineMotion(start, s, h, aMax, v1, v2);
		}
	}
	/**> 末端速度为0 */
	else
	{
		/**> 三段式停止规划 */
		if (fabs(v2 - v1) <= fabs(aMax*aMax/h))
		{
			aMax = sqrt(fabs((v2 - v1)*h));
			return threeLineMotion(start, s, h, aMax, v1);
		}
		/**> 四段式停止规划 */
		else
		{
			return fourLineMotion(start, s, h, aMax, v1);
		}
	}
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::query_flexible(double start, double s, double h, double aMax, double v1, double v2, double &realV2, bool stop) const
{
	if (stop && (fixZero(v2) != 0))
		return query_flexible_stop(start, s, h, aMax, v1, v2, realV2);
	println("SMPlannerEx: 柔性规划过渡段.");
	/**> 判断参数合理性 */
	if (s <= 0)
		throw("错误<SMPlannerEx>: 距离必须为正数!");
	if (fixZero(v1) < 0 || fixZero(v2) < 0)
		throw("错误<SMPlannerEx>: 速度必须为非负数!");
	if	(fixZero(v1) == 0 && fixZero(v2) == 0)
		throw("错误<SMPlannerEx>: 速度不能同时为0");
	/**> 如果始末速度相同, 则返回线性插补器 */
	if (fabs(v2 - v1) < 1e-10)
	{
		cout << "线性规划" << endl;
		Interpolator<double>::ptr interpolator0(new LinearInterpolator<double>(0, s, 2*s/(v1 + v2)));
		SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
		interpolator->addInterpolator(interpolator0);
		return interpolator;
	}
	/**> 末端速度不为0 */
	if (v2 > 0)
	{
		if (checkDitance(s, h, aMax, v1, v2, realV2));
		else
			v2 = realV2;
		/**> 三段式加速度 */
		if (fabs(v2 - v1) <= fabs(aMax*aMax/h))
		{
			aMax = sqrt(fabs((v2 - v1)*h));
			return threeLineMotion(start, s, h, aMax, v1, v2);
		}
		/**> 四段式加速度 */
		else
		{
			return fourLineMotion(start, s, h, aMax, v1, v2);
		}
	}
	/**> 末端速度为0 */
	else
	{
		if (checkDitance_stop(s, h, aMax, v1, v2, realV2));
		else
			v2 = realV2;
		/**> 三段式停止规划 */
		if (fabs(v2 - v1) <= fabs(aMax*aMax/h))
		{
			aMax = sqrt(fabs((v2 - v1)*h));
			return threeLineMotion(start, s, h, aMax, v1);
		}
		/**> 四段式停止规划 */
		else
		{
			return fourLineMotion(start, s, h, aMax, v1);
		}
	}
}

bool SMPlannerEx::checkDitance(double s, double h, double aMax, double v1, double v2, double &realV2, bool stop) const
{
	if (stop)
		return checkDitance_stop(s, h, aMax, v1, v2, realV2);
	if (s >= queryMinDistance(h, aMax, v1, v2))
	{
		realV2 = v2;
		return true;
	}
	else
	{
		realV2 = queryMaxSpeed(s, h, aMax, v1, v2);
		return false;
	}
}

double SMPlannerEx::queryMinDistance(double h, double aMax, double v1, double v2) const
{
	/**> 判断参数合理性 */
	if (fixZero(v1) < 0 || fixZero(v2) < 0)
		throw("错误<SMPlannerEx>: 速度必须为非负数!");
	/**> 如果始末速度相同 */
	if (fabs(v2 - v1) < 1e-12)
	{
		return 0;
	}
	/**> 三段式加速度 */
	if (fabs(v2 - v1) <= fabs(aMax*aMax/h))
	{
		aMax = sqrt(fabs((v2 - v1)*h));
		int sgn = (v2 > v1)? 1:-1;
		h = sgn*fabs(h);
		aMax = sgn*fabs(aMax);

		double t1 = aMax/h;
		double d2 = (v1 + v2)*t1;
		return d2;
	}
	/**> 四段式加速度 */
	else
	{
		double dv = v2 - v1;
		int sgn = (dv > 0)? 1:-1;
		h = sgn*fabs(h);
		aMax = sgn*fabs(aMax);

		double t3 = (v2 - v1)/aMax + aMax/h;
		double d3 = dv*dv/(2*aMax) + dv*aMax/(2*h) + v1*t3;
		return d3;
	}
}

double SMPlannerEx::queryMaxSpeed(double s, double h, double aMax, double v1, double v2) const
{
	if (s <= 0)
		throw("错误<SMPlannerEx>: 距离s必须大于0!");
	if (fixZero(v1) < 0 || fixZero(v2) < 0)
		throw("错误<SMPlannerEx>: 速度必须为非负数!");
	int sgn = (v2 > v1)? 1:-1;
	h = sgn*fabs(h);
	aMax = sgn*fabs(aMax);
	if (queryMinDistance(h, aMax, v1, v2) <= s)
		return v2;
	double lowerSpeed = v1;
	double upperSpeed = v2;
	double midSpeed = (lowerSpeed + upperSpeed)/2.0;
	double dv = upperSpeed - lowerSpeed;
	/**> 二分法查找最接近v2且可以达到的速度, v1是一定能达到的 */
	while(fabs(dv) >= 0.01)
	{
		if (queryMinDistance(h, aMax, v1, midSpeed) <= s)
			lowerSpeed = midSpeed;
		else
			upperSpeed = midSpeed;
		midSpeed = (lowerSpeed + upperSpeed)/2.0;
		dv = upperSpeed - lowerSpeed;
	}
	return lowerSpeed;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::query_stop(double start, double s, double h, double aMax, double v1, double v2) const
{
	double s1 = queryMinDistance(h, aMax, v1, v2);
	double s2 = queryMinDistance(h, aMax, v2, 0);
	double ds = s - s1 - s2;
	if (ds <= 0)
		throw ("错误<SMPlannerEx>: 停止段距离不够!");
	s1 += ds/2.0;
	s2 += ds/2.0;
	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(query(start, s1, h, aMax, v1, v2));
	interpolator->addInterpolator(query(start + s1, s2, h, aMax, v2, 0));
	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::query_flexible_stop(double start, double s, double h, double aMax, double v1, double v2, double &realV2) const
{
	println("SMPlannerEx: 柔性规划停止段.");
	if (checkDitance_stop(s, h, aMax, v1, v2, realV2));
	else
		v2 = realV2;
	double s1 = queryMinDistance(h, aMax, v1, v2);
	double s2 = queryMinDistance(h, aMax, v2, 0);
	double ds = s - s1 - s2;
	/** 最后一段的realV2可能也无法达到 */
	if (ds <= 0)
	{
		cout << "SMPlannerEx: 柔性规划: 停止段距离不够, 尝试直接减速" << endl;
		cout << " h = " << h << " aMax = " << aMax << " v1 = " << v1 << endl;
		cout << "最小距离: " << queryMinDistance(h, aMax, v1, 0) << endl;
		if (queryMinDistance(h, aMax, v1, 0) <= s)
		{
			return query(start, s, h, aMax, v1, 0);
		}
		else
			throw("错误<SMPlannerEx>: 柔性规划: 停止段距离不够!");
	}
	cout << "分成两部分进行规划" << endl;
	s1 += ds/2.0;
	s2 += ds/2.0;
	cout << "s1 = " << s1 << endl;
	cout << "s2 = " << s2 << endl;
	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(query(start, s1, h, aMax, v1, v2));
	interpolator->addInterpolator(query(start + s1, s2, h, aMax, v2, 0));
	return interpolator;
}

bool SMPlannerEx::checkDitance_stop(double s, double h, double aMax, double v1, double v2, double &realV2) const
{
	if (s >= queryMinDistance_stop(h, aMax, v1, v2))
	{
		realV2 = v2;
		return true;
	}
	else
	{
		realV2 = queryMaxSpeed_stop(s, h, aMax, v1, v2);
		return false;
	}
}

double SMPlannerEx::queryMinDistance_stop(double h, double aMax, double v1, double v2) const
{
	double s1 = queryMinDistance(h, aMax, v1, v2);
	double s2 = queryMinDistance(h, aMax, v2, 0);
	return s1 + s2;
}

double SMPlannerEx::queryMaxSpeed_stop(double s, double h, double aMax, double v1, double v2) const
{
	if (s <= 0)
		throw("错误<SMPlannerEx>: 距离s必须大于0!");
	if (fixZero(v1) < 0 || fixZero(v2) < 0)
		throw("错误<SMPlannerEx>: 速度必须为非负数!");
	if (queryMinDistance(h, aMax, v1, v2) <= s)
		return v2;
	double lowerSpeed = v1;
	double upperSpeed = v2;
	double midSpeed = (lowerSpeed + upperSpeed)/2.0;
	double dv = upperSpeed - lowerSpeed;
	/**> 二分法查找最接近v2且可以达到的速度, 假定v1是一定能达到的 */
	while(fabs(dv) >= 0.01)
	{
		if (queryMinDistance_stop(h, aMax, v1, midSpeed) <= s)
			lowerSpeed = midSpeed;
		else
			upperSpeed = midSpeed;
		midSpeed = (lowerSpeed + upperSpeed)/2.0;
		dv = upperSpeed - lowerSpeed;
	}
	return lowerSpeed;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::threeLineMotion(double start, double s, double h, double aMax, double v1, double v2) const
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

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(0 + start, v1, 0, h/6.0, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(d1 + start, h*t1*t1/2.0 + v1, h*t1/2.0, -h/6, t2 - t1));
	PolynomialInterpolator2<double>::ptr interpolator3(new PolynomialInterpolator2<double>(d2 + start, v2, 0, t3 - t2));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::fourLineMotion(double start, double s, double h, double aMax, double v1, double v2) const
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

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(0 + start, v1, 0, h/6.0, t1));
	PolynomialInterpolator2<double>::ptr interpolator2(new PolynomialInterpolator2<double>(d1 + start, t1*t1*h/2 + v1, aMax/2.0, t2 - t1));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(
			d2 + start, (v2 - h*pow(t3 - t2, 2)/2.0), h*(t3 - t2)/2.0, (-h/6.0), t3 - t2));
	PolynomialInterpolator2<double>::ptr interpolator4(new PolynomialInterpolator2<double>(d3 + start, v2, 0, t4 - t3));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::threeLineMotion(double start, double s, double h, double aMax, double v1) const
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

	PolynomialInterpolator2<double>::ptr interpolator0(new PolynomialInterpolator2<double>(0 + start, v1, 0, t0));
	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(d0 + start, v1, 0, h/6.0, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(d1 + start, h*t1*t1/2.0 + v1, h*t1/2.0, -h/6, t2 - t1));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator0);
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr SMPlannerEx::fourLineMotion(double start, double s, double h, double aMax, double v1) const
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

	PolynomialInterpolator2<double>::ptr interpolator0(new PolynomialInterpolator2<double>(0 + start, v1, 0, t0));
	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(d0 + start, v1, 0, h/6.0, t1));
	PolynomialInterpolator2<double>::ptr interpolator2(new PolynomialInterpolator2<double>(d1 + start, t1*t1*h/2 + v1, aMax/2.0, t2 - t1));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(
			d2 + start, (v2 - h*pow(t3 - t2, 2)/2.0), h*(t3 - t2)/2.0, (-h/6.0), t3 - t2));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator0);
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);

	return interpolator;
}

} /* namespace pathplanner */
} /* namespace robot */
