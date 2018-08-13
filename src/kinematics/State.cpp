/*
 * State.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

# include "State.h"
# include "../math/Q.h"

using robot::math::Q;

namespace robot {
namespace kinematic {

State::State(int size):
		_size(size)
{
	_state.push_back(Q::zero(size));
	_state.push_back(Q::zero(size));
	_state.push_back(Q::zero(size));
}

State::State(Q pos, Q vel, Q acc)
{
	_size = pos.size();
	_state.push_back(pos);
	_state.push_back(vel);
	_state.push_back(acc);
}

const robot::math::Q& State::getAngle() const
{
	return _state[0];
}

const robot::math::Q& State::getVelocity() const
{
	return _state[1];
}

const robot::math::Q& State::getAcceleration() const
{
	return _state[2];
}

const double State::getAngle(int jointNumber) const
{
	return _state[0][jointNumber];
}

const double State::getVelocity(int jointNumber) const
{
	return _state[1][jointNumber];
}

const double State::getAcceleration(int jointNumber) const
{
	return _state[2][jointNumber];
}

void State::setAngle(const robot::math::Q& angle)
{
	_state[0] = angle;
}

void State::setVelocity(const robot::math::Q& veloccity)
{
	_state[1] = veloccity;
}

void State::setAcceleration(const robot::math::Q& acceleration)
{
	_state[2] = acceleration;
}

void State::setAngle(double angle, int jointNumber)
{
	_state[0](jointNumber) = angle;
}
void State::setVelocity(double velocity, int jointNumber)
{
	_state[1](jointNumber) = velocity;
}

void State::setAcceleration(double acceleration, int jointNumber)
{
	_state[2](jointNumber) = jointNumber;
}

void State::operator=(const State &state)
{
	_size = state._size;
	_state = state._state;
}

State::~State()
{
}

} /* namespace kinematic */
} /* namespace robot */
