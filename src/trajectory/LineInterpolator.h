/*
 * LineInterpolator.h
 *
 *  Created on: Sep 20, 2017
 *      Author: a1994846931931
 */

#ifndef LINEINTERPOLATOR_H_
#define LINEINTERPOLATOR_H_

# include "ConvertedInterpolator.h"
# include "CompositeInterpolator.h"
# include <memory>

using namespace robot::trajectory;

namespace robot {
namespace trajectory {

/**
 * @brief 直线运动的插补器
 */
class LineInterpolator : public ikInterpolator{
public:
	using ptr = std::shared_ptr<LineInterpolator>;
	LineInterpolator(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
			std::shared_ptr<robot::ik::IKSolver> iksolver,
			robot::model::Config config,
			LinearCompositeInterpolator<double>::ptr mappedlt,
			LinearCompositeInterpolator<double>::ptr mappedtt);
	virtual ~LineInterpolator();
private:
	Interpolator<double>::ptr _lt;
	Interpolator<double>::ptr _tt;
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* LINEINTERPOLATOR_H_ */
