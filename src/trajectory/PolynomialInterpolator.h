/*
 * PolynomialInterpolator.h
 *
 *  Created on: Sep 4, 2017
 *      Author: a1994846931931
 */

#ifndef POLYNOMIALINTERPOLATOR_H_
#define POLYNOMIALINTERPOLATOR_H_

#include "Interpolator.h"

namespace robot {
namespace trajectory {

class PolynomialInterpolator: public Interpolator {
public:
	PolynomialInterpolator();
	virtual ~PolynomialInterpolator();
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* POLYNOMIALINTERPOLATOR_H_ */
