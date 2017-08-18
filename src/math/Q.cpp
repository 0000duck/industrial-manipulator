/*
 * Q.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

#include "Q.h"
# include "../common/printAdvance.h"

namespace robot {
namespace math {

Q::Q() {
	// TODO Auto-generated constructor stub
	_size = 0;
}

Q::Q(double q1, double q2, double q3, double q4, double q5, double q6)
{
	_size = 6;
	_value.push_back(q1);
	_value.push_back(q2);
	_value.push_back(q3);
	_value.push_back(q4);
	_value.push_back(q5);
	_value.push_back(q6);
}

//Q::Q(Q&)
//{
//
//}


int Q::size(){
	return _size;
}

double& Q::operator()(int index)
{
	return _value[index];
}

double& Q::operator[](int index)
{
	return _value[index];
}

Q& Q::zero(int size)
{
	static Q q(0, 0, 0, 0, 0, 0);
	return q;
}

void Q::print()
{
	for (int i = 0; i<_size; i++)
		cout << _value[i] << " ";
	robot::common::println();
}

Q::~Q() {
	// TODO Auto-generated destructor stub
}

} /* namespace math */
} /* namespace robot */
