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
# include <assert.h>
# include <math.h>

namespace robot {
namespace math {

template<typename T=double>
class HTransform3D {
public:
	HTransform3D() :
    _vec(),
    _rot(Rotation3D<T>::identity())
	{}

	HTransform3D(
			T r11, T r12, T r13, T r14,
			T r21, T r22, T r23, T r24,
			T r31, T r32, T r33, T r34) :
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

	const HTransform3D<T> operator* (const HTransform3D<T>& tran) const
	{
		return HTransform3D<T>(_rot*tran._vec + _vec, _rot*tran._rot);
	}

	const HTransform3D<T> inverse()
	{
		return HTransform3D<T>(-(_rot.inverse()*_vec), _rot.inverse());
	}

	const T operator()(int row, int col) const
	{
		assert(row < 3);
		assert(col < 4);
		if (col < 3)
			return _rot(row, col);
		else
			return _vec(row);
	}

	inline void setRotation(
			const T& r11, const T& r12, const T& r13,
			const T& r21, const T& r22, const T& r23,
			const T& r31, const T& r32, const T& r33)
	{
		_rot.setRotation(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	}

	inline void setVector(const T& v1, const T& v2, const T& v3)
	{
		_vec.setVector(v1, v2, v3);
	}

	inline const Rotation3D<T>& getRotation() const
	{
		return _rot;
	}

	inline const Vector3D<T>& getPosition() const
	{
		return _vec;
	}

	inline void update(
			const T& r11, const T& r12, const T& r13, const T& r14,
			const T& r21, const T& r22, const T& r23, const T& r24,
			const T& r31, const T& r32, const T& r33, const T& r34)
	{
		_rot.setRotation(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		_vec.setVector(r14, r24, r34);
	}

	static const HTransform3D<T>& identity(){
		static const HTransform3D<T> id(
			Vector3D<T>(0, 0, 0),
			Rotation3D<T>::identity());
		return id;
	}

	/*
	 * 根据DH参数(Modified DH)构造HTransform3D；
	 */
	static const HTransform3D<T>& DH(T alpha, T a, T d, T theta)
	{
		double st=sin(theta); double ct=cos(theta);double sa=sin(alpha);double ca=cos(alpha);
		double a11=ct;double a12=-st;double a13=0;double a14=a;
		double a21=st*ca;double a22=ct*ca;double a23=-sa;double a24=-d*sa;
		double a31=st*sa;double a32=ct*sa;double a33=ca;double a34=d*ca;

		static robot::math::Rotation3D<double> rot(
													a11, a12, a13,
													a21, a22, a23,
													a31, a32, a33);
		static robot::math::Vector3D<double> vec(a14, a24, a34);
		static robot::math::HTransform3D<double> tran(vec, rot);
		return tran;
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
