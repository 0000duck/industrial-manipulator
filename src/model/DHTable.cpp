/*
 * DHTable.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

# include "DHTable.h"
# include <iostream>
namespace robot {
namespace model {

DHTable::DHTable() {

}


int DHTable::size()
{
	return _dHParam.size();
}

const DHParameters& DHTable::operator()(int index) const
{
	return _dHParam[index];
}

const DHParameters& DHTable::operator[](int index) const
{
	return _dHParam[index];
}

void DHTable::append(const DHParameters& dHParam)
{
	_dHParam.push_back(dHParam);
}

void DHTable::print() const
{
	std::cout << "alpha\ta\td\ttheta" << std::endl;
	for (int i=0; i<(int)_dHParam.size(); i++)
	{
		_dHParam[i].print();
	}
}

DHTable::~DHTable() {
}

} /* namespace model */
} /* namespace robot */
