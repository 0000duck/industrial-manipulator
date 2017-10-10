/*
 * Config.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: a1994846931931
 */

#include "Config.h"
# include "../common/printAdvance.h"

using namespace robot::common;

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

bool Config::operator==(const Config& config) const
{
	if (_shoulder == config.getShoulder() && _elbow == config.getElbow() && _wrist == config.getWrist())
		return true;
	else
		return false;
}

bool Config::operator!=(const Config& config) const
{
	return !(this->operator ==(config));
}

void Config::print() const
{
	switch(_shoulder)
	{
	case Config::righty:
		cout << "righty\t";
		break;
	case Config::lefty:
		cout << "lefty\t";
		break;
	case Config::ssame:
		cout << "ssame\t";
		break;
	case Config::sfree:
		cout << "sfree\t";
		break;
	}
	switch(_elbow)
	{
	case Config::epositive:
		cout << "epositive\t";
		break;
	case Config::enegative:
		cout << "enegative\t";
		break;
	case Config::esame:
		cout << "esame\t";
		break;
	case Config::efree:
		cout << "efree\t";
		break;
	}
	switch(_wrist)
	{
	case Config::wpositive:
		cout << "wpositive\t";
		break;
	case Config::wnegative:
		cout << "wnegative\t";
		break;
	case Config::wsame:
		cout << "wsame\t";
		break;
	case Config::wfree:
		cout << "wfree\t";
		break;
	}
	println();
}

Config::~Config() {
}

} /* namespace model */
} /* namespace robot */
