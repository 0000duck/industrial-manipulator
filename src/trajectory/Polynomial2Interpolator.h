/*
 * Polynomial2Interpolator.h
 *
 *  Created on: Sep 4, 2017
 *      Author: a1994846931931
 */

#ifndef POLYNOMIAL2INTERPOLATOR_H_
#define POLYNOMIAL2INTERPOLATOR_H_

#include "Interpolator.h"

namespace robot {
namespace trajectory {

class Polynomial2Interpolator: public Interpolator {
public:
	Polynomial2Interpolator();
	virtual ~Polynomial2Interpolator();
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* POLYNOMIAL2INTERPOLATOR_H_ */
