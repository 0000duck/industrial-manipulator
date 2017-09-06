/*
 * Jacobian.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: a1994846931931
 */

# include "Jacobian.h"
# include "../ext/Eigen/Dense"
# include "../common/printAdvance.h"

namespace robot {
namespace model {

Jacobian::Jacobian():_j(6, 6) // 默认关节数为6
{
	_size = 6;
}

Jacobian::Jacobian(std::vector< std::vector<double> > j):_j(6, j[0].size())
{
	if (j.size() != 6)
		throw("错误: 用于雅克比矩阵初始化的数组大小必须为6! ");
	_size = j[0].size();
	for (int i=0; i<6; i++)
	{
		for (int k=0; k<_size; k++)
		{
			_j(i, k) = j[i][k];
		}
	}
}


void Jacobian::doInverse()
{
	if (_size != 6)
		throw("Jacobian is not inversible because the #joint is not 6! ");
	_j = _j.inverse();
}

Jacobian Jacobian::inverse() const
{
	if (_size != 6)
		throw("Jacobian is not inversible because the #joint is not 6! ");
	Jacobian jcob = *this;
	jcob.getMatrix().inverse();
	return jcob;
}

int Jacobian::rank() const
{
	Eigen::ColPivHouseholderQR<Eigen::MatrixXd > QR_decomp(_j);
	return QR_decomp.rank();
}

void Jacobian::print() const
{
	cout.precision(4);
	cout << setfill('_') << setw(14*6) <<  "_" << endl;
	for (int i=0; i<6; i++)
	{
		for (int j=0; j<6; j++)
		{
			cout << setfill(' ') << setw(12) << _j(i, j) << " |";
		}
		cout << endl;
	}
}

void Jacobian::update(std::vector< std::vector<double> > j)
{
	if (j.size() != _size)
		throw("错误: 赋值的数组大小与被赋值的雅克比矩阵大小不同! ");
	for (int i=0; i<6; i++)
	{
		for (int k=0; k<_size; k++)
		{
			_j(i, k) = j[i][k];
		}
	}
}

double Jacobian::operator()(int a, int b) const
{
	return _j(a, b);
}

void Jacobian::operator=(const Jacobian& jaco)
{
	for (int i=0; i<6; i++)
	{
		for (int k=0; k<6; k++)
		{
			_j(i, k) = jaco(i, k);
		}
	}
}

robot::math::Q Jacobian::operator*(const robot::math::Q& jointVelocity) const
{
	if (jointVelocity.size() != _size)
		throw("Jacobian doesn't match joint velocity size");
	robot::math::Q endVelocity = robot::math::Q::zero(6);
	for (int i=0; i<6; i++)
	{
		for (int j=0; j<_size; j++)
		{
			endVelocity(i) += _j(i, j)*jointVelocity[j];
		}
	}
	return endVelocity;
}

int Jacobian::size() const
{
	return _size;
}

Jacobian::~Jacobian()
{
}


} /* namespace model */
} /* namespace robot */
