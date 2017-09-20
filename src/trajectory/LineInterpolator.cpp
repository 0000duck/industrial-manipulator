/*
 * LineInterpolator.cpp
 *
 *  Created on: Sep 20, 2017
 *      Author: a1994846931931
 */

#include "LineInterpolator.h"

namespace robot {
namespace trajectory {

LineInterpolator::LineInterpolator(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
		std::shared_ptr<robot::ik::IKSolver> iksolver,
		robot::model::Config config,
		LinearCompositeInterpolator<double>::ptr mappedlt,
		LinearCompositeInterpolator<double>::ptr mappedtt)
:ikInterpolator(origin, iksolver, config), _lt(mappedlt), _tt(mappedtt)
{

}

LineInterpolator::~LineInterpolator() {
	// TODO Auto-generated destructor stub
}

} /* namespace trajectory */
} /* namespace robot */
