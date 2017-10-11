/*
 * ExcessMotionPlanner.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: zrf
 */

# ifdef COMPILEEXCESS

# include "ExcessMotionPlanner.h"
# include "math.h"
# include "../trajectory/PolynomialInterpolator.h"
# include "../trajectory/SequenceInterpolator.h"
# include "../common/printAdvance.h"

using namespace robot::common;

namespace robot {
namespace pathplanner {

using namespace robot::trajectory;


ExcessMotionPlanner::ExcessMotionPlanner() {

}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::query(double s, double vMax, double aMax, double j, double ve, double vs,double start ) const
{

}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::Motion7(double s, double vMax, double aMax, double j, double ve, double vs,double start) const
{
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double st1 = this -> st1(j, vMax, vs, aMax);
	double st2 = this -> st2(j, vMax, ve, aMax);
	double t1 =  aMax/j;
	double t3 = t1;
	double t5 = t1;
	double t7 = t1;
	double t2 = (vMax - vs)/aMax - t1;
	double t4 = (s - st1 - st2)/vMax;
	double t6 = (vMax - ve)/aMax - t5;

	double d1 =	t1*t1*t1*j/6 + vs*t1;
	double d2 = d1 + vs*t2 + j*t1*t1*t2/2 + aMax*t2*t2/2;
	double d3 = st1;
	double d4 = s - st2;
	double d5 = d4 + vMax*t5 - j*t5*t5*t5/6;
	double d6 = d5 + ve*t6 + j*t7*t7*t6/2 + aMax*t6*t6/2;
	double d7 = s;

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(start, vs, 0, sign*j/6, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(sign*d1 + start, sign*(vs + j*t1*t1/2), sign*aMax/2, 0, t2));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(sign*d2 + start, sign*(vMax - j*t3*t3/2), sign*j*t3/2, sign*(-j/6), t3));
	PolynomialInterpolator3<double>::ptr interpolator4(new PolynomialInterpolator3<double>(sign*d3 + start, sign*vMax, 0, 0, t4));
	PolynomialInterpolator3<double>::ptr interpolator5(new PolynomialInterpolator3<double>(sign*d4 + start, sign*vMax, 0, sign*(-j/6), t5));
	PolynomialInterpolator3<double>::ptr interpolator6(new PolynomialInterpolator3<double>(sign*d5 + start, sign*(ve + j*t7*t7/2), sign*aMax/2, 0, t6));
	PolynomialInterpolator3<double>::ptr interpolator7(new PolynomialInterpolator3<double>(sign*(s - j*t7*t7*t7/6 + ve*t7) + start, sign*(ve + t7*t7*j/2), sign*(-j*t7), sign*j/6, t7));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);
	interpolator->addInterpolator(interpolator6);
	interpolator->addInterpolator(interpolator7);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::Motion6(double s, double vMax, double aMax, double j, double ve, double vs,double start) const
{
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double st1 = this -> st1(j, vMax, vs, aMax);
	double st2 = this -> st2(j, vMax, ve, aMax);
	double t1 = aMax/j;
	double t3 = t1;
	double t4 = t1;
	double t6 = t1;
	double t2 = (vMax - vs)/aMax - t1;
	double t5 = (vMax - ve)/aMax - t4;

	double d1 =	t1*t1*t1*j/6 + vs*t1;
	double d2 = d1 + vs*t2 + j*t1*t1*t2/2 + aMax*t2*t2/2;
	double d3 = st1;
	double d4 = d3 + vMax*t4 - j*t4*t4*t4/6;
	double d5 = d4 + ve*t5 + j*t6*t6*t5/2 + aMax*t5*t5/2;
	double d6 = s;

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(start, vs, 0, sign*j/6, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(sign*d1 + start, sign*(vs + j*t1*t1/2), sign*aMax/2, 0, t2));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(sign*d2 + start, sign*(vMax - j*t3*t3/2), sign*j*t3/2, sign*(-j/6), t3));
	PolynomialInterpolator3<double>::ptr interpolator4(new PolynomialInterpolator3<double>(sign*d4 + start, sign*vMax, 0, sign*(-j/6), t4));
	PolynomialInterpolator3<double>::ptr interpolator5(new PolynomialInterpolator3<double>(sign*d5 + start, sign*(ve + j*t6*t6/2), sign*aMax/2, 0, t5));
	PolynomialInterpolator3<double>::ptr interpolator6(new PolynomialInterpolator3<double>(sign*(s - j*t6*t6*t6/6 + ve*t6) + start, sign*(ve + t6*t6*j/2), sign*(-j*t6), sign*j/6, t6));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);
	interpolator->addInterpolator(interpolator6);

	return interpolator;

}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::Motion22(double s, double vMax, double aMax, double j, double ve, double vs,double start) const
{
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double ss1 = this -> ss1(j, vMax, vs, aMax);
	double ss2 = this -> ss2(j, vMax, ve, aMax);
	double t1 = sqrt((vMax-vs)/j);
	double t2 = t1;
	double t3 = sqrt((vMax-ve)/j);
	double t4 = t3;

	double d1 =	t1*t1*t1*j/6 + vs*t1;
	double d2 = ss1;
	double d3 = s - ve*t4 - t4*t4*t4*j/6;
	double d4 = s;

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(start, vs, 0, sign*j/6, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(sign*d1 + start, sign*(vMax - j*t2*t2/2), sign*j*t2/2, sign*(-j/6), t2));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(sign*d2 + start, sign*vMax, 0, sign*(-j/6), t3));
	PolynomialInterpolator3<double>::ptr interpolator4(new PolynomialInterpolator3<double>(sign*(s - j*t4*t4*t4/6 + ve*t4) + start, sign*(ve + t4*t4*j/2), sign*(-j*t4), sign*j/6, t4));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);

	return interpolator;

}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::Motion32(double s, double vMax, double aMax, double j, double ve, double vs,double start) const
{
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double st1 = this ->st1(j, vMax, vs, aMax);
	double ss2 = this ->ss2(j, vMax, ve, aMax);
	double t1 = aMax/j;
	double t3 = t1;
	double t4 = sqrt((vMax-ve)/j);
	double t5 = t4;
	double t2 = (vMax - vs)/aMax - t1;

	double d1 =	t1*t1*t1*j/6 + vs*t1;
	double d2 = d1 + vs*t2 + j*t1*t1*t2/2 + aMax*t2*t2/2;
	double d3 = st1;
	double d4 = s - ve*t5 - t5*t5*t5*j/6;
	double d5 = s;

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(start, vs, 0, sign*j/6, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(sign*d1 + start, sign*(vs + j*t1*t1/2), sign*aMax/2, 0, t2));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(sign*d2 + start, sign*(vMax - j*t3*t3/2), sign*j*t3/2, sign*(-j/6), t3));
	PolynomialInterpolator3<double>::ptr interpolator4(new PolynomialInterpolator3<double>(sign*d3 + start, sign*vMax, 0, sign*(-j/6), t4));
	PolynomialInterpolator3<double>::ptr interpolator5(new PolynomialInterpolator3<double>(sign*(s - j*t5*t5*t5/6 + ve*t5) + start, sign*(ve + t5*t5*j/2), sign*(-j*t5), sign*j/6, t5));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::Motion23(double s, double vMax, double aMax, double j, double ve, double vs,double start) const
{
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double ss1 = this ->ss1(j, vMax, vs, aMax);
	double st2 = this ->st2(j, vMax, ve, aMax);
	double t1 = sqrt((vMax-vs)/j);
	double t2 = t1;
	double t3 = aMax/j;
	double t5 = t3;
	double t4 = (vMax - ve)/aMax - t5;

	double d1 =	t1*t1*t1*j/6 + vs*t1;
	double d2 = ss1;
	double d3 = d2 + vMax*t3 - j*t3*t3*t3/6;
	double d4 = d3 + ve*t4 + j*t5*t5*t4/2 + aMax*t4*t4/2;
	double d5 = s;

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(start, vs, 0, sign*j/6, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(sign*d1 + start, sign*(vs + j*t1*t1/2), sign*aMax/2, 0, t2));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(sign*d2 + start, sign*vMax, 0, sign*(-j/6), t3));
	PolynomialInterpolator3<double>::ptr interpolator4(new PolynomialInterpolator3<double>(sign*d3 + start, sign*(ve + j*t5*t5/2), sign*aMax/2, 0, t4));
	PolynomialInterpolator3<double>::ptr interpolator5(new PolynomialInterpolator3<double>(sign*(s - j*t5*t5*t5/6 + ve*t5) + start, sign*(ve + t5*t5*j/2), sign*(-j*t5), sign*j/6, t5));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::Motion302(double s, double vMax, double aMax, double j, double ve, double vs,double start) const
{
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double st1 = this ->st1(j, vMax, vs, aMax);
	double ss2 = this ->ss2(j, vMax, ve, aMax);
	double t1 = aMax/j;
	double t3 = t1;
	double t2 = (vMax - vs)/aMax - t1;
	double t4 = (s - st1 - ss2)/vMax;
	double t5 = sqrt((vMax-ve)/j);
	double t6 = t5;

	double d1 =	t1*t1*t1*j/6 + vs*t1;
	double d2 = d1 + vs*t2 + j*t1*t1*t2/2 + aMax*t2*t2/2;
	double d3 = st1;
	double d4 = s - ss2;
	double d5 = s - ve*t5 - t5*t5*t5*j/6;
	double d6 = s;

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(start, vs, 0, sign*j/6, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(sign*d1 + start, sign*(vs + j*t1*t1/2), sign*aMax/2, 0, t2));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(sign*d2 + start, sign*(vMax - j*t3*t3/2), sign*j*t3/2, sign*(-j/6), t3));
	PolynomialInterpolator3<double>::ptr interpolator4(new PolynomialInterpolator3<double>(sign*d3 + start, sign*vMax, 0, 0, t4));
	PolynomialInterpolator3<double>::ptr interpolator5(new PolynomialInterpolator3<double>(sign*d4 + start, sign*vMax, 0, sign*(-j/6), t5));
	PolynomialInterpolator3<double>::ptr interpolator6(new PolynomialInterpolator3<double>(sign*(s - j*t6*t6*t6/6 + ve*t6) + start, sign*(ve + t6*t6*j/2), sign*(-j*t6), sign*j/6, t6));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);
	interpolator->addInterpolator(interpolator6);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::Motion203(double s, double vMax, double aMax, double j, double ve, double vs,double start) const
{
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double ss1 = this ->ss1(j, vMax, vs, aMax);
	double st2 = this ->st2(j, vMax, ve, aMax);
	double t1 = sqrt((vMax-vs)/j);
	double t2 = t1;
	double t3 = (s - ss1 - st2)/vMax;
	double t4 = aMax/j;
	double t6 = t4;
	double t5 = (vMax - ve)/aMax - t4;

	double d1 =	t1*t1*t1*j/6 + vs*t1;
	double d2 = ss1;
	double d3 = s - st2;
	double d4 = d3 + vMax*t4 - j*t4*t4*t4/6;
	double d5 = d4 + ve*t5 + j*t6*t6*t5/2 + aMax*t5*t5/2;
	double d6 = s;

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(start, vs, 0, sign*j/6, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(sign*d1 + start, sign*(vs + j*t1*t1/2), sign*aMax/2, 0, t2));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(sign*d2 + start, sign*vMax, 0, 0, t3));
	PolynomialInterpolator3<double>::ptr interpolator4(new PolynomialInterpolator3<double>(sign*d3 + start, sign*vMax, 0, sign*(-j/6), t4));
	PolynomialInterpolator3<double>::ptr interpolator5(new PolynomialInterpolator3<double>(sign*d4 + start, sign*(ve + j*t6*t6/2), sign*aMax/2, 0, t5));
	PolynomialInterpolator3<double>::ptr interpolator6(new PolynomialInterpolator3<double>(sign*(s - j*t6*t6*t6/6 + ve*t6) + start, sign*(ve + t6*t6*j/2), sign*(-j*t6), sign*j/6, t6));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);
	interpolator->addInterpolator(interpolator6);

	return interpolator;
}

robot::trajectory::SequenceInterpolator<double>::ptr ExcessMotionPlanner::Motion202(double s, double vMax, double aMax, double j, double ve, double vs,double start) const
{
	int sign = (s < 0)? -1:1;
	s = fabs(s);
	double ss1 = this ->st1(j, vMax, vs, aMax);
	double ss2 = this ->st2(j, vMax, ve, aMax);
	double t1 = sqrt((vMax-vs)/j);
	double t2 = t1;
	double t3 = (s - ss1 - ss2)/vMax;
	double t4 = sqrt((vMax-ve)/j);
	double t5 = t4;

	double d1 =	t1*t1*t1*j/6 + vs*t1;
	double d2 = ss1;
	double d3 = s - ss2;
	double d4 = s - ve*t5 - t5*t5*t5*j/6;
	double d5 = s;

	PolynomialInterpolator3<double>::ptr interpolator1(new PolynomialInterpolator3<double>(start, vs, 0, sign*j/6, t1));
	PolynomialInterpolator3<double>::ptr interpolator2(new PolynomialInterpolator3<double>(sign*d1 + start, sign*(vs + j*t1*t1/2), sign*aMax/2, 0, t2));
	PolynomialInterpolator3<double>::ptr interpolator3(new PolynomialInterpolator3<double>(sign*d2 + start, sign*vMax, 0, 0, t3));
	PolynomialInterpolator3<double>::ptr interpolator4(new PolynomialInterpolator3<double>(sign*d3 + start, sign*vMax, 0, sign*(-j/6), t4));
	PolynomialInterpolator3<double>::ptr interpolator5(new PolynomialInterpolator3<double>(sign*(s - j*t5*t5*t5/6 + ve*t5) + start, sign*(ve + t5*t5*j/2), sign*(-j*t5), sign*j/6, t5));

	SequenceInterpolator<double>::ptr interpolator(new SequenceInterpolator<double>());
	interpolator->addInterpolator(interpolator1);
	interpolator->addInterpolator(interpolator2);
	interpolator->addInterpolator(interpolator3);
	interpolator->addInterpolator(interpolator4);
	interpolator->addInterpolator(interpolator5);

	return interpolator;
}

ExcessMotionPlanner::~ExcessMotionPlanner() {

}

double ExcessMotionPlanner::st1(double j, double vMax, double vs, double aMax) const
{
	return ((j*(vMax*vMax - vs*vs) + aMax*aMax*(vMax + vs))/2/aMax/j);
}

double ExcessMotionPlanner::st2(double j, double vMax, double ve,double aMax) const
{
	return ((j*(vMax*vMax - ve*ve) + aMax*aMax*(vMax + ve))/2/aMax/j);
}

double ExcessMotionPlanner::ss1(double j, double vMax, double vs, double aMax) const
{
	return ((vMax + vs)*sqrt((vMax - vs)/j));
}

double ExcessMotionPlanner::ss2(double j, double vMax, double ve, double aMax) const
{
	return ((vMax + ve)*sqrt((vMax - ve)/j));
}

}
}

# endif


