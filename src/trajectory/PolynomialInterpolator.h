/**
 * @brief PolynomialInterpolator类
 * @date Sep 4, 2017
 * @author a1994846931931
 */

#ifndef POLYNOMIALINTERPOLATOR_H_
#define POLYNOMIALINTERPOLATOR_H_

#include "Interpolator.h"

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */
template <class T>
class PolynomialInterpolator: public Interpolator<T> {
public:
	PolynomialInterpolator();
	virtual ~PolynomialInterpolator();
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* POLYNOMIALINTERPOLATOR_H_ */
