/*
 * common.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: a1994846931931
 */

# include "common.h"

namespace robot{
namespace common{

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
}
}

