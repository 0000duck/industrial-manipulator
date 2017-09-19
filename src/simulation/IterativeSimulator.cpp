/*
 * IterativeSimulator.cpp
 *
 *  Created on: Sep 19, 2017
 *      Author: a1994846931931
 */

#include "IterativeSimulator.h"

namespace robot {
namespace simulation {

IterativeSimulator::IterativeSimulator(SerialLink* robot)
{
	_robot = robot;
	_position = robot->getQ();
	_velocity = Q::zero(robot->getDOF());
	_accelaration = Q::zero(robot->getDOF());
}

const State IterativeSimulator::getState()
{
	return State(_position, _velocity, _accelaration);
}

void IterativeSimulator::setSpeed(Q velocity, double duration)
{
	_position += _velocity*duration; //backward
	_position += velocity*duration; //forward
	_position += (_velocity + velocity)*(duration/2.0); //
	_accelaration = (velocity - _velocity)*(1.0/duration);
	_velocity = velocity;
}

} /* namespace simulation */
} /* namespace robot */
