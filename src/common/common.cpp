/*
 * common.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: a1994846931931
 */

# include "common.h"
# include <stdlib.h>
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
}
}

