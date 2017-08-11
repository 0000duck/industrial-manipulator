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
	static const HTransform3D& identity(){
		static const HTransform3D id(
			Vector3D<T>(0, 0, 0),
			Rotation3D<T>::identity());
		return id;
	}
	void print() const{
		for (int i = 0; i<3; i++)
			cout<<_rot(i, 0)<<" "<<_rot(i, 1)<<" "<<_rot(i, 2)<<" "<<_vec(i)<<'\n';
		cout<<"0 0 0 1";
	}
	virtual ~HTransform3D(){}
private:
	Vector3D<T> _vec;
	Rotation3D<T> _rot;
};

} /* namespace math */
} /* namespace robot */

#endif /* HTRANSFORM3D_H_ */
