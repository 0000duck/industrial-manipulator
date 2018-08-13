/*
 * IterativeSimulator.cpp
 *
 *  Created on: Sep 19, 2017
 *      Author: a1994846931931
 */

#include "IterativeSimulator.h"

namespace robot {
namespace simulation {

IterativeSimulator::IterativeSimulator(SerialLink::ptr robot)
{
	_robot = robot;
	_position = robot->getQ();
	_velocity = Q::zero(robot->getDOF());
	_acceleration = Q::zero(robot->getDOF());
	_duration = 0;
}

const State IterativeSimulator::getState()
{
	Q position = _position;
	Q velocity = _velocity;
	Q acceleration = _acceleration;
	return State(position, velocity, acceleration);
}

void IterativeSimulator::setSpeed(Q velocity, double duration)
{
	int mode = 1;
	switch (mode)
	{
	case 0: _position += _velocity*duration; break;//backward
	case 1: _position += velocity*duration; break; //forward
	default: _position += (_velocity + velocity)*(duration/2.0); break;//
	}
	_acceleration = (velocity - _velocity)*(1.0/duration);
	_velocity = velocity;
	_duration += duration;
}

double IterativeSimulator::getDuration() const
{
	return _duration;
}

} /* namespace simulation */
} /* namespace robot */
