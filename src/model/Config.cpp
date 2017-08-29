/*
 * Config.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: a1994846931931
 */

#include "Config.h"

namespace robot {
namespace model {

Config::Config():
		_shoulder(0),
		_elbow(0),
		_wrist(0)
{

}

Config::Config(int shoulder, int elbow, int wrist):
		_shoulder(shoulder),
		_elbow(elbow),
		_wrist(wrist)
{

}

void Config::setShoulder(int shoulder)
{
	_shoulder = shoulder;
}

void Config::setElbow(int elbow)
{
	_elbow = elbow;
}

void Config::setWrist(int wrist)
{
	_wrist = wrist;
}

int Config::getShoulder() const
{
	return _shoulder;
}

int Config::getElbow() const
{
	return _elbow;
}

int Config::getWrist() const
{
	return _wrist;
}

Config::~Config() {
}

} /* namespace model */
} /* namespace robot */
