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

//Q::Q(Q& q)
//{
//	_size = q.size();
//	for (int i=0; i<_size; i++)
//		_value.push_back(q[i]);
//}


int Q::size() const
{
	return _size;
}

double& Q::operator()(int index)
{
	return _value[index];
}

double Q::operator[](int index) const
{
	return _value[index];
}

Q Q::operator+(const Q& q1) const
{
	if (this->size() != q1.size())
		throw("错误: 尝试将不同大小的数组相加!");
	Q q(*this);
	for (int i=0; i<(int)_size; i++)
		q(i) += q1[i];
	return q;
}

Q Q::operator-(const Q& q1) const
{
	if (this->size() != q1.size())
		throw("错误: 尝试将不同大小的数组相减!!");
	Q q(*this);
	for (int i=0; i<(int)_size; i++)
		q(i) -= q1[i];
	return q;
}

Q Q::operator*(double num) const
{
	Q q(*this);
	for (int i=0; i<this->size(); i++)
		q(i) *= num;
	return q;
}

Q Q::operator/(double num) const
{
	double factor = 1.0/num;
	Q q(*this);
	for (int i=0; i<this->size(); i++)
		q(i) *= factor;
	return q;
}

Q Q::zero(int size)
{
	Q q;
	for (int i=0; i<size; i++)
		q.pushBack(0);
	return q;
}

void Q::pushBack(double newValue)
{
	_size++;
	_value.push_back(newValue);
}

void Q::print() const
{
	cout.precision(4);
	for (int i = 0; i<_size; i++)
		cout << _value[i] << " || ";
	robot::common::println();
}

Q::~Q() {
}

} /* namespace math */
} /* namespace robot */
