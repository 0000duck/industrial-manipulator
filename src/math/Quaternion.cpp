/*
 * Quaternion.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: a1994846931931
 */

# include "Quaternion.h"
# include <math.h>
# include "../common/printAdvance.h"
# include "../common/common.h"

namespace robot {
namespace math {

using namespace robot::common;

Quaternion::Quaternion():_r(1),_i(0),_j(0),_k(0)
{
}

Quaternion::Quaternion(double r,double i,double j,double k):_r(r),_i(i),_j(j),_k(k)
{

}

Quaternion::Quaternion(const Quaternion& QuatA)
{
	_r=QuatA._r;
	_i=QuatA._i;
	_j=QuatA._j;
	_k=QuatA._k;
}

Quaternion::Quaternion(const Rotation3D<double>& rot)
{
	_r = 0.5*sqrt(fixZero(1 + rot(0, 0) + rot(1, 1) + rot(2, 2)));
	double _r4 = _r*4;
	if (fabs(_r4) > 1e-12)
	{
		_i = (rot(2, 1) - rot(1, 2))/_r4;
		_j = (rot(0, 2) - rot(2, 0))/_r4;
		_k = (rot(1, 0) - rot(0, 1))/_r4;
	}
	else
	{
		_i = sqrt(fixZero((rot(0, 0) + 1.0)/2.0));
		_j = sqrt(fixZero((rot(1, 1) + 1.0)/2.0));
		_k = sqrt(fixZero((rot(2, 2) + 1.0)/2.0));
	}
}

Quaternion::Quaternion(const HTransform3D<double>& tran)
{
	_r = 0.5*sqrt(1 + tran(0, 0) + tran(1, 1) + tran(2, 2));
	double _r4 = _r*4;
	_i = (tran(2, 1) - tran(1, 2))/_r4;
	_j = (tran(0, 2) - tran(2, 0))/_r4;
	_k = (tran(1, 0) - tran(0, 1))/_r4;
}

Quaternion::Quaternion(double theta, const Vector3D<double>& n)
{
	double st = sin(theta/2.0);
	_r = cos(theta/2.0);
	_i = n(0)*st;
	_j = n(1)*st;
	_k = n(2)*st;
}

Quaternion Quaternion::operator+(const Quaternion& QuatA) const
{
	return Quaternion(_r+QuatA._r,_i+QuatA._i,_j+QuatA._j,_k+QuatA._k);
}
Quaternion Quaternion::operator-(const Quaternion& QuatA) const
{
	return Quaternion(_r-QuatA._r,_i-QuatA._i,_j-QuatA._j,_k-QuatA._k);
}
Quaternion Quaternion::operator*(const Quaternion& QuatA) const
{

	return Quaternion(_r*QuatA._r-_i*QuatA._i-_j*QuatA._j-_k*QuatA._k
			,_r*QuatA._i+_i*QuatA._r+_j*QuatA._k-_k*QuatA._j
			,_r*QuatA._j+_j*QuatA._r+_k*QuatA._i-_i*QuatA._k
			,_r*QuatA._k+_k*QuatA._r+_i*QuatA._j-_j*QuatA._i);
}
void Quaternion::operator+=(const Quaternion& QuatA)
{
	_r+=QuatA._r;
	_i+=QuatA._i;
	_j+=QuatA._j;
	_k+=QuatA._k;
}
void Quaternion::operator-=(const Quaternion& QuatA)
{
	_r-=QuatA._r;
	_i-=QuatA._i;
	_j-=QuatA._j;
	_k-=QuatA._k;
}
void Quaternion::operator*=(const Quaternion& QuatA)
{
	double r = _r*QuatA._r-_i*QuatA._i-_j*QuatA._j-_k*QuatA._k;
	double i = _r*QuatA._i+_i*QuatA._r+_j*QuatA._k-_k*QuatA._j;
	double j = _r*QuatA._j+_j*QuatA._r+_k*QuatA._i-_i*QuatA._k;
	double k = _r*QuatA._k+_k*QuatA._r+_i*QuatA._j-_j*QuatA._i;
	_r = r;
	_i = i;
	_j = j;
	_k = k;
}
void Quaternion::operator=(const Quaternion& QuatA)
{
	_r=QuatA._r;
	_i=QuatA._i;
	_j=QuatA._j;
	_k=QuatA._k;
}
bool Quaternion::operator==(const Quaternion& QuatA) const
{
	if (_r==QuatA._r&&_i==QuatA._i&&_j==QuatA._j&&_k==QuatA._k)
	{
		return true;
	}
	else return false;
}
bool Quaternion::operator!=(const Quaternion& QuatA) const
{
	if (_r==QuatA._r&&_i==QuatA._i&&_j==QuatA._j&&_k==QuatA._k)
		{
			return false;
		}
		else return true;
}
double Quaternion::r() const
{return _r;}
double Quaternion::i() const
{return _i;}
double Quaternion::j() const
{return _j;}
double Quaternion::k() const
{return _k;}
Quaternion Quaternion::conjugate() const
{
	return Quaternion(_r,-_i,-_j,-_k);
}
double Quaternion::norm() const
{
	return sqrt(_r*_r+_i*_i+_j*_j+_k*_k);
}
void Quaternion::normalize()
{
	double q=norm();
	_r = _r/q;
	_i = _i/q;
	_j = _j/q;
	_k = _k/q;
}
robot::math::Rotation3D<double> Quaternion::toRotation3D() const
{
	return Rotation3D<double> (1-2*_j*_j-2*_k*_k,   2*_i*_j+2*_r*_k,    2*_i*_k-2*_r*_j,
			                   2*_i*_j-2*_r*_k,     1-2*_i*_i-2*_k*_k,  2*_j*_k+2*_r*_i,
			                   2*_i*_k+2*_r*_j,     2*_j*_k-2*_r*_i,    1-2*_i*_i-2*_j*_j);
}
robot::math::HTransform3D<double> Quaternion::toHTransform3D() const
{
	return HTransform3D<double> (1-2*_j*_j-2*_k*_k,   2*_i*_j+2*_r*_k,    2*_i*_k-2*_r*_j,  0,
				                 2*_i*_j-2*_r*_k,     1-2*_i*_i-2*_k*_k,  2*_j*_k+2*_r*_i,  0,
				                 2*_i*_k+2*_r*_j,     2*_j*_k-2*_r*_i,    1-2*_i*_i-2*_j*_j,0);
}

Quaternion::rotVar Quaternion::getRotationVariables() const
{
	double theta = acos(_r)*2;
	double st = sin(theta/2);
	Quaternion::rotVar var;
	var.theta = theta;
	if (fabs(st) > 1e-12)
		var.n = Vector3D<double>(_i/st, _j/st, _k/st);
	else
		var.n = Vector3D<double>(0, 0, 0);
	return var;
}

void Quaternion::print() const
{
	cout << _r << " + " << _i << "i + " << _j << "j + " << _k << "k" << endl;
}
Quaternion Quaternion::DH(double alpha, double theta)
{
	double ca = cos(alpha/2);
	double sa = sin(alpha/2);
	double ct = cos(theta/2);
	double st = sin(theta/2);
	return Quaternion(ca*ct, sa*ct, -sa*st, ca*st);
}


Quaternion::~Quaternion() {
	// TODO Auto-generated destructor stub
}

} /* namespace math */
} /* namespace robot */
