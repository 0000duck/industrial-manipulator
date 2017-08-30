/*
 * State.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

# include "State.h"
# include "../math/Q.h"

namespace robot {
namespace kinematic {

State::State(int size):
		_size(size)
{
	_jointAngle = robot::math::Q::zero(size);
	_jointVelocity = robot::math::Q::zero(size);
	_jointAccleration = robot::math::Q::zero(size);
}
const robot::math::Q& State::getAngle() const
{
	return _jointAngle;
}

const robot::math::Q& State::getVelocity() const
{
	return _jointVelocity;
}

const robot::math::Q& State::getAcceleration() const
{
	return _jointAccleration;
}

const double State::getAngle(int jointNumber) const
{
	return _jointAngle[jointNumber];
}

const double State::getVelocity(int jointNumber) const
{
	return _jointVelocity[jointNumber];
}

const double State::getAcceleration(int jointNumber) const
{
	return _jointAccleration[jointNumber];
}

void State::setAngle(const robot::math::Q& angle)
{
	_jointAngle = angle;
}

void State::setVelocity(const robot::math::Q& veloccity)
{
	_jointVelocity = veloccity;
}

void State::setAcceleration(const robot::math::Q& acceleration)
{
	_jointAccleration = acceleration;
}

void State::setAngle(double angle, int jointNumber)
{
	_jointAngle(jointNumber) = angle;
}
void State::setVelocity(double velocity, int jointNumber)
{
	_jointVelocity(jointNumber) = velocity;
}

void State::setAcceleration(double acceleration, int jointNumber)
{
	_jointAccleration(jointNumber) = jointNumber;
}

State::~State()
{
}

} /* namespace kinematic */
} /* namespace robot */
