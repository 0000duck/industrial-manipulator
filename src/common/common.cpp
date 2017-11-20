/*
 * common.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: a1994846931931
 */

# include "common.h"
# include <stdlib.h>
# include <sys/time.h>
# include <math.h>

namespace robot{
namespace common{

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

double fixAngle(double angle)
{
	double fix = fmod((angle + M_PI), 2*M_PI);
	return  (fix < 0)? (fix + M_PI):(fix - M_PI);
}

double fixZero(double num)
{
	if (num < 0 && num > -1e-12)
		return 0;
	return num;
}

unsigned long long getUTime()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec*1000000 + tv.tv_usec;
}

double min_d(double a, double b) { return ((a < b)? a:b);}

double max_d(double a, double b) { return ((a > b)? a:b);}

}
}

