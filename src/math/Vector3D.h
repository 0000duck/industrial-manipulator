/*
 * Vector3D.h
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

#ifndef VECTOR3D_H_
#define VECTOR3D_H_

# include "Rotation3D.h"
# include <iostream>

namespace robot {
namespace math {

template<typename T=double>
class Vector3D {
public:
	Vector3D()
	{
		_v[0] = _v[1] = _v[2] = 0;
	}
	Vector3D(T v1, T v2, T v3)
	{
		_v[0] = v1;
		_v[1] = v2;
		_v[2] = v3;
	}
	Vector3D(const Vector3D<T>& vSource)
	{
		_v[0] = vSource(0);
		_v[1] = vSource(1);
		_v[2] = vSource(2);
	}
	double getLengh() const
	{
		return sqrt(_v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]);
	}
	const T operator()(int i) const
	{
		return _v[i];
	}
	T& operator[](int i) const
	{
		return _v[i];
	}
	void operator=(const Vector3D<T>& vSource)
	{
		_v[0] = vSource(0);
		_v[1] = vSource(1);
		_v[2] = vSource(2);
	}
	bool operator==(const Vector3D<T>& vSource)
	{
		if (_v[0]!=vSource(0) or _v[1]!=vSource(1) or _v[2]!=vSource(2))
			return false;
		return true;
	}
	bool operator!=(const Vector3D<T>& vSource)
	{
		return (!this->operator ==(vSource));
	}
	Vector3D<T> operator-() const
	{
		return Vector3D<T>(
				-_v[0], -_v[1], -_v[2]);
	}
	Vector3D<T> operator+(Vector3D<T> vec) const
	{
		return Vector3D<T>(
				vec(0) + _v[0], vec(1) + _v[1], vec(2) + _v[2]);
	}
	static T dot(const Vector3D<T>& a, const Vector3D<T>& b)
	{
		return (a(0)*b(0) + a(1)*b(1) + a(2)*b(2));
	}
	void setVector(const T& v1, const T& v2, const T& v3)
	{
		_v[0] = v1;
		_v[1] = v2;
		_v[2] = v3;
	}
//	static Rotation3D<T>& cross(const Vector3D<T>& a, const Vector3D<T>& b){
//		static Rotation3D<T> result(a(0))
//	}
	virtual ~Vector3D(){}
private:
	T _v[3];
};

} /* namespace math */
} /* namespace robot */

#endif /* VECTOR3D_H_ */
