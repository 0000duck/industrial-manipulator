/*
 * State.h
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

#ifndef STATE_H_
#define STATE_H_

# include "../math/Q.h"

namespace robot {
namespace kinematic {

class State {
public:
	State(int size);
	const robot::math::Q& getAngle() const;
	const robot::math::Q& getVelocity() const;
	const robot::math::Q& getAcceleration() const;
	const double getAngle(int jointNumber) const;
	const double getVelocity(int jointNumber) const;
	const double getAcceleration(int jointNumber) const;
	void setAngle(const robot::math::Q&);
	void setVelocity(const robot::math::Q&);
	void setAcceleration(const robot::math::Q&);
	void setAngle(double angle, int jointNumber);
	void setVelocity(double velocity, int jointNumber);
	void setAcceleration(double acceleration, int jointNumber);
	~State();
private:
	int _size;
	robot::math::Q _jointAngle;
	robot::math::Q _jointVelocity;
	robot::math::Q _jointAccleration;
private:

};

} /* namespace kinematic */
} /* namespace robot */

#endif /* STATE_H_ */
