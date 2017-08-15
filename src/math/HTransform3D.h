/*
 * HTransform3D.h
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

#ifndef HTRANSFORM3D_H_
#define HTRANSFORM3D_H_

# include "Rotation3D.h"
# include "Vector3D.h"
# include <iostream>

namespace robot {
namespace math {

template<typename T>
class HTransform3D {
public:
	HTransform3D() :
    _vec(),
    _rot(Rotation3D<T>::identity())
	{}
	HTransform3D(
			T r11, T r12, T r13, T r14,
			T r21, T r22, T r23, T r24,
			T r31, T r32, T r33, T r34,
			T r41, T r42, T r43, T r44) :
	_vec(r14, r24, r34),
	_rot(r11, r12, r13, r21, r22, r23, r31, r32, r33)
	{}
	HTransform3D(const Vector3D<T>& d, const Rotation3D<T>& R) :
	_vec(d),
	_rot(R)
	{}
	explicit HTransform3D(const Rotation3D<T>& R) :
	_vec(0, 0, 0),
	_rot(R)
	{}
	explicit HTransform3D(const Vector3D<T>& d) :
	_vec(d),
	_rot(Rotation3D<T>::identity())
	{}
	inline void setRotation(
			const T& r11, const T& r12, const T& r13,
			const T& r21, const T& r22, const T& r23,
			const T& r31, const T& r32, const T& r33)
	{
		_rot.setRotation(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	}
	static const HTransform3D& identity(){
		static const HTransform3D id(
			Vector3D<T>(0, 0, 0),
			Rotation3D<T>::identity());
		return id;
	}
	void print() const{
		for (int i = 0; i<3; i++)
			cout<<_rot(i, 0)<<" "<<_rot(i, 1)<<" "<<_rot(i, 2)<<" "<<_vec(i)<<'\n';
		cout<<"0 0 0 1"<<'\n';
	}
	virtual ~HTransform3D(){}
private:
	Vector3D<T> _vec;
	Rotation3D<T> _rot;
};

} /* namespace math */
} /* namespace robot */

#endif /* HTRANSFORM3D_H_ */
