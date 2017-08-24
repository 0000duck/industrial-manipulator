/*
 * Quaternion.h
 *
 *  Created on: Aug 24, 2017
 *      Author: a1994846931931
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

# include "Rotation3D.h"
# include "HTransform3D.h"

namespace robot {
namespace math {

class Quaternion {
public:
	Quaternion();
	Quaternion(double, double, double, double);
	Quaternion(const Quaternion&);
	Quaternion operator+(const Quaternion&) const;
	Quaternion operator-(const Quaternion&) const;
	Quaternion operator*(const Quaternion&) const;
	void operator+=(const Quaternion&);
	void operator-=(const Quaternion&);
	void operator*=(const Quaternion&);
	void operator=(const Quaternion&);
	bool operator==(const Quaternion&) const;
	bool operator!=(const Quaternion&) const;
	double r() const;
	double i() const;
	double j() const;
	double k() const;
	Quaternion conjugate() const;
	double norm() const;
	robot::math::Rotation3D<double> toRotation3D() const;
	robot::math::HTransform3D<double> toHTransform3D() const;
	void print();
	virtual ~Quaternion();
private:
	double _r, _i, _j, _k;
};

} /* namespace math */
} /* namespace robot */

#endif /* QUATERNION_H_ */
