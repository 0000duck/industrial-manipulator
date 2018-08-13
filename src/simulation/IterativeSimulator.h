/*
 * IterativeSimulator.h
 *
 *  Created on: Sep 19, 2017
 *      Author: a1994846931931
 */

#ifndef ITERATIVESIMULATOR_H_
#define ITERATIVESIMULATOR_H_

# include "../model/SerialLink.h"
# include "../kinematics/State.h"
# include "../math/Q.h"

using robot::model::SerialLink;
using robot::kinematic::State;
using robot::math::Q;

namespace robot {
namespace simulation {

class IterativeSimulator {
public:
	IterativeSimulator(SerialLink::ptr robot);
	const State getState();
	void setSpeed(Q velocity, double duration);
	double getDuration() const;
	virtual ~IterativeSimulator(){}
private:
	SerialLink::ptr _robot;
	Q _position;
	Q _velocity;
	Q _acceleration;
	double _duration;
};

} /* namespace simulation */
} /* namespace robot */

#endif /* ITERATIVESIMULATOR_H_ */
