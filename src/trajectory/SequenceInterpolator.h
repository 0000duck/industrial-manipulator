/**
 * @brief SequenceInterpolator类
 * @date Sep 6, 2017
 * @author a1994846931931
 */

#ifndef SEQUENCEINTERPOLATOR_H_
#define SEQUENCEINTERPOLATOR_H_

#include "Interpolator.h"

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @{
 */

/**
 *
 */
class SequenceInterpolator: public Interpolator {
public:
	SequenceInterpolator();
	virtual ~SequenceInterpolator();
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* SEQUENCEINTERPOLATOR_H_ */
