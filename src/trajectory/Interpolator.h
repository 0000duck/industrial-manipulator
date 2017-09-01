/*
 * Interpolator.h
 *
 *  Created on: Sep 1, 2017
 *      Author: a1994846931931
 */

#ifndef INTERPOLATOR_H_
#define INTERPOLATOR_H_

namespace robot {
namespace trajectory {

template <class T>
class Interpolator {
public:
	Interpolator(){}
	virtual ~Interpolator(){}
	virtual T x(double t) const = 0;
	virtual T dx(double t) const = 0;
	virtual T ddx(double t) const = 0;
	virtual double duration() const = 0;
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* INTERPOLATOR_H_ */
