/*
 * CircularTrajectory.h
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */

#ifndef CIRCULARTRAJECTORY_H_
#define CIRCULARTRAJECTORY_H_

# include "ConvertedInterpolator.h"
# include "CompositeInterpolator.h"
# include "Trajectory.h"
# include <memory>

using namespace robot::trajectory;

namespace robot {
namespace trajectory {

class CircularTrajectory : public Interpolator<Q>{
public:
	CircularTrajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
			std::shared_ptr<robot::ik::IKSolver> iksolver,
			robot::model::Config config,
			SequenceInterpolator<double>::ptr lt,
			Trajectory::ptr trajectory);

	Q x(double t) const;
	Q dx(double t) const;
	Q ddx(double t) const;
	double l(double t) const;
	double dl(double t) const;
	double ddl(double t) const;
	double duration() const;
	virtual ~CircularTrajectory(){}
private:
	ikInterpolator::ptr _qIpr;
	Trajectory::ptr _trajectory;

	/** @brief 直线距离-时间插补器 */
	SequenceInterpolator<double>::ptr _lt;

	std::vector<std::pair<double, double> > _lengthPath;
	const int _pathSize;
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* CIRCULARTRAJECTORY_H_ */
