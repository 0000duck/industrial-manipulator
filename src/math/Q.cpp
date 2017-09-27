/*
 * Q.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

#include "Q.h"
# include "../common/printAdvance.h"
# include <algorithm>

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

Q Q::operator*(const Q& q1) const
{
	if (this->size() != q1.size())
		throw("错误: 尝试将不同大小的数组相乘!!");
	Q q(*this);
	for (int i=0; i<(int)_size; i++)
		q(i) *= q1[i];
	return q;
}

Q Q::operator/(const Q& q1) const
{
	if (this->size() != q1.size())
		throw("错误: 尝试将不同大小的数组相除!!");
	Q q(*this);
	for (int i=0; i<(int)_size; i++)
		q(i) = q(i)/q1[i];
	return q;
}

Q Q::operator+(double num) const
{
	Q q(*this);
	for (int i=0; i<this->size(); i++)
		q(i) += num;
	return q;
}

Q Q::operator-(double num) const
{
	Q q(*this);
	for (int i=0; i<this->size(); i++)
		q(i) -= num;
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

void Q::operator+=(const Q& q)
{
	if (this->size() != q.size())
		throw("错误: 尝试将不同大小的数组相乘!!");
	for (int i=0; i<this->size(); i++)
		_value[i] += q[i];
}

void Q::operator-=(const Q& q)
{
	if (this->size() != q.size())
		throw("错误: 尝试将不同大小的数组相乘!!");
	for (int i=0; i<this->size(); i++)
		_value[i] -= q[i];
}

void Q::operator*=(const Q& q)
{
	if (this->size() != q.size())
		throw("错误: 尝试将不同大小的数组相乘!!");
	for (int i=0; i<this->size(); i++)
		_value[i] *= q[i];
}

void Q::operator/=(const Q& q)
{
	if (this->size() != q.size())
		throw("错误: 尝试将不同大小的数组相乘!!");
	for (int i=0; i<this->size(); i++)
		_value[i] /= q[i];
}

void Q::operator+=(double num)
{
	for (int i=0; i<this->size(); i++)
		_value[i] += num;
}

void Q::operator-=(double num)
{
	for (int i=0; i<this->size(); i++)
		_value[i] -= num;
}

void Q::operator*=(double num)
{
	for (int i=0; i<this->size(); i++)
		_value[i] *= num;
}

void Q::operator/=(double num)
{
	for (int i=0; i<this->size(); i++)
		_value[i] /= num;
}

Q Q::zero(int size)
{
	Q q;
	for (int i=0; i<size; i++)
		q.pushBack(0);
	return q;
}

bool Q::operator==(const Q& q) const
{
	if (_size != q.size())
		return false;
	for (int i=0; i<_size; i++)
	{
		if (_value[i] != q[i])
			return false;
	}
	return true;
}


bool Q::operator!=(const Q& q) const
{
	return !(this->operator ==(q));
}

bool Q::operator<(const Q& q) const
{
	for (int i=0; i<_size; i++)
	{
		if (_value[i] >= q[i])
			return false;
	}
	return true;
}

bool Q::operator<=(const Q& q) const
{
	for (int i=0; i<_size; i++)
	{
		if (_value[i] > q[i])
			return false;
	}
	return true;
}

bool Q::operator>(const Q& q) const
{
	for (int i=0; i<_size; i++)
	{
		if (_value[i] <= q[i])
			return false;
	}
	return true;
}

bool Q::operator>=(const Q& q) const
{
	for (int i=0; i<_size; i++)
	{
		if (_value[i] < q[i])
			return false;
	}
	return true;
}

void Q::pushBack(double newValue)
{
	_size++;
	_value.push_back(newValue);
}

void Q::abs()
{
	for_each(_value.begin(), _value.end(), [](double &value){value = fabs(value);});
}


void Q::doMinmax(Q&min, Q&max)
{
	for (int i=0; i<_size; i++)
	{
		if (_value[i] < min[i])
		{
			min(i) = _value[i];
		}
		else if (_value[i] > max[i])
		{
			max(i) = _value[i];
		}
	}
}

void Q::doMin(Q&min)
{
	for (int i=0; i<_size; i++)
	{
		if (_value[i] < min[i])
		{
			min(i) = _value[i];
		}
	}
}

void Q::doMax(Q&max)
{
	for (int i=0; i<_size; i++)
	{
		if (_value[i] > max[i])
		{
			max(i) = _value[i];
		}
	}
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
