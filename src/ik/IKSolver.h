/*
 * IKSolver.h
 *
 *  Created on: Sep 11, 2017
 *      Author: a1994846931931
 */

#ifndef IKSOLVER_H_
#define IKSOLVER_H_

# include "../math/Q.h"
# include "../math/HTransform3D.h"
# include "../model/Config.h"

using namespace robot::math;

namespace robot {
namespace ik {

class IKSolver {
public:
	IKSolver(){}
	virtual std::vector<Q> solve(const HTransform3D<>& baseTend, const model::Config& config) const = 0;
	virtual ~IKSolver();
};

} /* namespace ik */
} /* namespace robot */

#endif /* IKSOLVER_H_ */
