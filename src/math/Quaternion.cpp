/*
 * Quaternion.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: a1994846931931
 */

#include "Quaternion.h"
#include <math.h>

namespace robot {
namespace math {

Quaternion::Quaternion():_r(0),_i(0),_j(0),_k(0)
{
	// TODO Auto-generated constructor stub
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
	_r*=QuatA._r;
	_i*=QuatA._i;
	_j*=QuatA._j;
	_k*=QuatA._k;
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
Quaternion Quaternion::norm() const
{
	double q=sqrt(_r*_r+_i*_i+_j*_j+_k*_k);
	return Quaternion(_r/q,_i/q,_j/q,_k/q);
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
Quaternion::~Quaternion() {
	// TODO Auto-generated destructor stub
}

} /* namespace math */
} /* namespace robot */
