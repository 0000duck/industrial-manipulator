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
# include "../model/SerialLink.h"

using namespace robot::math;

namespace robot {
namespace ik {

/**
 * @addtogroup ik
 * @{
 */

/**
 * @brief 逆解器基类
 */
class IKSolver {
public:
	IKSolver() {}
	virtual std::vector<Q> solve(const HTransform3D<>& baseTend, const model::Config& config) const = 0;
	virtual robot::model::Config getConfig(const robot::math::Q& q) const = 0;
	virtual robot::model::SerialLink::ptr getRobot() = 0;
	virtual ~IKSolver();
};

/**@}*/

} /* namespace ik */
} /* namespace robot */

#endif /* IKSOLVER_H_ */
