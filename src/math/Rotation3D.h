/*
 * Rotation3D.h
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

#ifndef ROTATION3D_H_
#define ROTATION3D_H_

# include <iostream>
# include <limits>
# include <math.h>
# include "Vector3D.h"

using std::cout;

namespace robot {
namespace math {

template<class T=double>
class Rotation3D {
public:
	Rotation3D(){
		_m[0][0] = 1;
		_m[0][1] = 0;
		_m[0][2] = 0;
		_m[1][0] = 0;
		_m[1][1] = 1;
		_m[1][2] = 0;
		_m[2][0] = 0;
		_m[2][1] = 0;
		_m[2][2] = 1;
	}
	Rotation3D(
		const T r11, const T r12, const T r13,
		const T r21, const T r22, const T r23,
		const T r31, const T r32, const T r33){
		_m[0][0] = r11;
		_m[0][1] = r12;
		_m[0][2] = r13;
		_m[1][0] = r21;
		_m[1][1] = r22;
		_m[1][2] = r23;
		_m[2][0] = r31;
		_m[2][1] = r32;
		_m[2][2] = r33;
	}
	Rotation3D(
			const Vector3D<T>& i,
			const Vector3D<T>& j,
			const Vector3D<T>& k){
		_m[0][0] = i[0];
		_m[0][1] = j[0];
		_m[0][2] = k[0];
		_m[1][0] = i[1];
		_m[1][1] = j[1];
		_m[1][2] = k[1];
		_m[2][0] = i[2];
		_m[2][1] = j[2];
		_m[2][2] = k[2];
	}
	void setRotation(
			T r11, T r12, T r13,
			T r21, T r22, T r23,
			T r31, T r32, T r33)
	{
		_m[0][0] = r11;
		_m[0][1] = r12;
		_m[0][2] = r13;
		_m[1][0] = r21;
		_m[1][1] = r22;
		_m[1][2] = r23;
		_m[2][0] = r31;
		_m[2][1] = r32;
		_m[2][2] = r33;
	}
	static const Rotation3D& identity()
	{
		static Rotation3D id(1,0,0,0,1,0,0,0,1);
		return id;
	}
	void print(){
		cout<<_m[0][0]<<" "<<_m[0][1]<<" "<<_m[0][2]<<'\n';
		cout<<_m[1][0]<<" "<<_m[1][1]<<" "<<_m[1][2]<<'\n';
		cout<<_m[2][0]<<" "<<_m[2][1]<<" "<<_m[2][2]<<'\n';
	}
	inline const T& operator()(int row, int column) const{
		return _m[row][column];
	}
//	inline T& operator[](int row, int column) const{
//			return _m[row][column];
//	}
	bool operator==(const Rotation3D<T>& RotB){
		for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
				if (_m[i][j] != RotB(i, j))
					return false;
		return true;
	}
	bool operator!=(const Rotation3D<T>& RotB){
		return !(*this == RotB);
	}
	bool equal(const Rotation3D<T>& rot, const T precision = std::numeric_limits<T>::epsilon()) const {
		for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
				if (fabs(_m[i][j] - rot(i,j)) > precision)
					return false;
		return true;
	}
	static Rotation3D<T> multiply(const Rotation3D<T>& a, const Rotation3D<T>& b){
		return Rotation3D<T>(
				a(0, 0)*b(0, 0) + a(0, 1)*b(1, 0) + a(0, 2)*b(2, 0),
				a(0, 0)*b(0, 1) + a(0, 1)*b(1, 1) + a(0, 2)*b(2, 1),
				a(0, 0)*b(0, 2) + a(0, 1)*b(1, 2) + a(0, 2)*b(2, 2),

				a(1, 0)*b(0, 0) + a(1, 1)*b(1, 0) + a(1, 2)*b(2, 0),
				a(1, 0)*b(0, 1) + a(1, 1)*b(1, 1) + a(1, 2)*b(2, 1),
				a(1, 0)*b(0, 2) + a(1, 1)*b(1, 2) + a(1, 2)*b(2, 2),

				a(2, 0)*b(0, 0) + a(2, 1)*b(1, 0) + a(2, 2)*b(2, 0),
				a(2, 0)*b(0, 1) + a(2, 1)*b(1, 1) + a(2, 2)*b(2, 1),
				a(2, 0)*b(0, 2) + a(2, 1)*b(1, 2) + a(2, 2)*b(2, 2));
	}
	Rotation3D<T> operator*(const Rotation3D<T> rot) const
	{
		return multiply(*this, rot);
	}
	Vector3D<T> operator*(const Vector3D<T> vec) const
	{
		return Vector3D<T>(
				_m[0][0]*vec(0) + _m[0][1]*vec(1) + _m[0][2]*vec(2),
				_m[1][0]*vec(0) + _m[1][1]*vec(1) + _m[1][2]*vec(2),
				_m[2][0]*vec(0) + _m[2][1]*vec(1) + _m[2][2]*vec(2));
	}
	Rotation3D<T> inverse() const
	{
		return Rotation3D<T>(
				_m[0][0], _m[1][0], _m[2][0],
				_m[0][1], _m[1][1], _m[2][1],
				_m[0][2], _m[1][2], _m[2][2]);
	}
//	friend HTransform3D<T>::setRotation(T, T, T, T, T, T, T, T, T);
	virtual ~Rotation3D(){}
private:
	T _m[3][3];
};

} /* namespace math */
} /* namespace robot */

#endif /* ROTATION3D_H_ */
