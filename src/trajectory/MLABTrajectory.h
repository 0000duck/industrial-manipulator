/*
 * MLABTrajectory.h
 *
 *  Created on: Sep 26, 2017
 *      Author: a1994846931931
 */

#ifndef MLABTRAJECTORY_H_
#define MLABTRAJECTORY_H_

#include "Interpolator.h"
# include "../math/Q.h"
# include <memory>

using robot::math::Q;

namespace robot {
namespace trajectory {

class MLABTrajectory: public Interpolator<Q> {
public:
	using ptr = std::shared_ptr<MLABTrajectory>;

	MLABTrajectory();
	virtual ~MLABTrajectory();
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* MLABTRAJECTORY_H_ */
